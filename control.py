'''
Central logic for the daemon.
'''
from io import TextIOWrapper
from mavlink import MavlinkConnectionManager, MavlinkTelemetryMonitor, TYPE_RSSI_GLOBAL, TYPE_VICTIM_MARKER
from wifi_rssi import WifiRssiMonitor
from DataPoint import DataPoint, DataPointCollection
from processing import location_interpolate, determine_final_victims
from time import sleep, time
from os import makedirs, path
from datetime import datetime
import struct
import logging

logger = logging.getLogger(__name__) 
logger.level = logging.DEBUG

class Control: 
    '''
    Overall program logic.
    '''
    _mav: MavlinkConnectionManager 
    _telem: MavlinkTelemetryMonitor
    _wifi_mon: WifiRssiMonitor
    _log_dir: str
    '''directory to store files'''
    _data_file: TextIOWrapper
    '''raw data file'''
    _victim_file: TextIOWrapper 
    '''victim location file'''
    _collection: DataPointCollection
    '''collection of data points for this mission'''
    _mission_dir: str
    '''directory for current mission logs'''
    _last_processed_rssi_time: float
    '''timestamp of last processed RSSI value (to detect new beacon pulses)'''

    _local_position_file: TextIOWrapper 
    _global_position_file: TextIOWrapper 
    _wifi_rssi_file: TextIOWrapper

    __mission_started_flag: bool
    __mission_ended_flag: bool

    def __init__(
            self, 
            mav_manager: MavlinkConnectionManager, 
            mav_telemetry: MavlinkTelemetryMonitor, 
            wifi_monitor: WifiRssiMonitor, 
            log_directory: str
            ) -> None: 
        '''
        Constructor.
        
        Args:
            mav_manager: MAVLink connection manager
            mav_telemetry: MAVLink telemetry monitor
            wifi_monitor: WiFi RSSI monitor
            log_directory: Base directory for log files
        '''
        self._mav = mav_manager 
        self._telem = mav_telemetry
        self._wifi_mon = wifi_monitor
        self._log_dir = log_directory
        self._collection = DataPointCollection()
        self._last_processed_rssi_time = 0.0

        self._mav.mission_started_cb = self.__mission_started_handler
        self._mav.mission_ended_cb = self.__mission_ended_handler
        self.__mission_started_flag = False 
        self.__mission_ended_flag = False

    def run(
            self 
            ) -> None: 
        '''
        Main program.
        '''

        # TODO potentially check if mission has started before we could wait on the event

        # loop indefinitely (can do more than one mission)
        while True:

            # wait for start of mission
            logger.info("Waiting for start of mission.")
            while True: 
                if self.__mission_started_flag: 
                    self.__mission_started_flag = False
                    break
                sleep(0.25)

            # progress through the states
            logger.info("Mission starting.")
            self.__mision_init()
            logger.info("Mission initialized.")
            self.__run_mission()
            logger.info("Mission complete.")
            self.__mission_deinit()
            logger.info("Mission deinitialized.")

    def __mission_started_handler(
            self
            ) -> None: 
        '''
        Handle mission started event.
        '''
        self.__mission_started_flag = True

    def __mission_ended_handler(
            self
            ) -> None: 
        '''
        Handle mission ended event.
        '''
        self.__mission_ended_flag = True

    def __get_next_mission_number(self) -> int:
        '''
        Get the next mission number using a persistent counter file.
        This works even if the Raspberry Pi doesn't have a real time clock.
        
        Returns:
            int: Next mission number
        '''
        counter_file = path.join(self._log_dir, ".mission_counter")
        
        try:
            # Try to read existing counter
            if path.exists(counter_file):
                with open(counter_file, "r") as f:
                    counter = int(f.read().strip())
            else:
                counter = 0
        except (ValueError, IOError):
            # If file is corrupted or can't be read, start from 0
            counter = 0
        
        # Increment counter
        counter += 1
        
        # Write back to file
        try:
            makedirs(self._log_dir, exist_ok=True)
            with open(counter_file, "w") as f:
                f.write(str(counter))
        except IOError:
            # If we can't write, still use the counter but warn
            logger.warning(f" Could not write mission counter file, using counter={counter}")
        
        return counter

    def __mision_init(
            self
            ) -> None:
        '''
        Run once when mission starts.
        Creates mission directory with persistent counter-based name for easy identification.
        '''
        # Get next mission number (persists across reboots)
        mission_num = self.__get_next_mission_number()
        
        # Create mission directory with zero-padded number for easy sorting
        # Format: mission_001, mission_002, etc.
        mission_name = f"mission_{mission_num:03d}"
        self._mission_dir = path.join(self._log_dir, mission_name)
        makedirs(self._mission_dir, exist_ok=True)
        
        # Open log files in write mode
        self._data_file = open(path.join(self._mission_dir, "signal_map.dat"), "wb")
        self._victim_file = open(path.join(self._mission_dir, "victim.txt"), "w")

        self._global_position_file = open(path.join(self._mission_dir, "global_positions.csv"), "w") 
        self._local_position_file = open(path.join(self._mission_dir, "local_positions.csv"), "w")
        self._wifi_rssi_file = open(path.join(self._mission_dir, "wifi_rssi.csv"), "w")

        # headers for csv file 
        self._global_position_file.write("latitude,longitude,time\n")
        self._local_position_file.write("x,y,time\n")
        self._wifi_rssi_file.write("rssi,time\n")

        self._telem.start_logging(
                self._local_position_file, 
                self._global_position_file
                ) 
        self._wifi_mon.start_logging(
                self._wifi_rssi_file
                )
        
        # Clear collection for new mission
        self._collection.clear()
        self._last_processed_rssi_time = 0.0  # Reset for new mission

        # clear telemetry data 
        self._telem.clear()
        
        # Log mission start
        logger.info(f"Mission #{mission_num} started\n")

    def __run_mission(
            self
            ) -> None:
        '''
        Runs during the main mission.
        Interpolates location, creates data points, sends tunnel messages, and logs data.
        '''
        rssi_check_window = 1  # seconds: short window to detect new pulses
        
        while True:
            # check for end of mission.
            if self.__mission_ended_flag:
                self.__mission_ended_flag = False 
                break 

            # Detect the latest/max RSSI in the recent past
            rssi_time = self._wifi_mon.get_max_rssi(rssi_check_window)
            
            if rssi_time is not None:
                # Check if this is a new beacon pulse (timestamp is newer than last processed)
                if rssi_time.time > self._last_processed_rssi_time:
                    # This is a new beacon pulse - process it immediately
                    self._last_processed_rssi_time = rssi_time.time
                    
                    # Get location data from telemetry (need at least 2 points for interpolation)
                    if len(self._telem._lat_list) >= 2:
                        # Interpolate location for the RSSI timestamp
                        try:
                            point = location_interpolate(
                                self._telem._lat_list,
                                self._telem._lon_list,
                                self._telem._global_time_list,
                                rssi_time
                            )
                            
                            # Add to collection
                            self._collection.add_point(point)
                            
                            # Send tunnel message to base station
                            self._mav._send_tunnel_message(
                                point.latitude,
                                point.longitude,
                                point.rssi,
                                TYPE_RSSI_GLOBAL
                            )
                            
                            # Write to data file (binary format: lat, lon, rssi as floats)
                            data_bytes = struct.pack('<fff', point.latitude, point.longitude, point.rssi)
                            self._data_file.write(data_bytes)
                            self._data_file.flush()
                            
                            # Log message
                            logger.info(f"DataPoint (beacon pulse): lat={point.latitude:.6f}, lon={point.longitude:.6f}, rssi={point.rssi:.1f} dBm")
                            
                        except Exception as e:
                            logger.error(f"Error processing data point: {e}\n")
                    else:
                        # Not enough location data yet - skip this pulse
                        pass
            
            # Small sleep to prevent waiting while still being responsive to pulses
            sleep(rssi_check_window) 


    def __mission_deinit(
            self
            ) -> None: 
        '''
        Run once when mission ends.
        Calculates victim location, sends final marker, and writes summary files.
        '''

        self._telem.stop_logging()
        self._wifi_mon.stop_logging()
        
        # Calculate victim locations using Monte Carlo hill climbing and support filtering
        if len(self._collection) > 0:
            victims = determine_final_victims(self._collection.data_points, max_victims=3)
            
            if victims:
                # Write all detected victims to file
                self._victim_file.write(f"Detected {len(victims)} victim(s):\n")
                self._victim_file.write("=" * 60 + "\n\n")
                
                # Send victim markers and write to file
                for i, (victim_lat, victim_lon, victim_rssi) in enumerate(victims, 1):
                    # Send victim marker
                    self._mav._send_tunnel_message(
                        victim_lat,
                        victim_lon,
                        victim_rssi,
                        TYPE_VICTIM_MARKER
                    )
                    
                    # Write victim location to file
                    self._victim_file.write(f"Victim {i}:\n")
                    self._victim_file.write(f"  Latitude: {victim_lat:.6f}\n")
                    self._victim_file.write(f"  Longitude: {victim_lon:.6f}\n")
                    self._victim_file.write(f"  RSSI: {victim_rssi:.1f} dBm\n")
                    self._victim_file.write("\n")
                
                # Also write strongest single point for reference
                self._victim_file.write(f"\nStrongest single point:\n")
                strongest = self._collection.strongest()
                if strongest:
                    self._victim_file.write(f"Latitude: {strongest.latitude:.6f}\n")
                    self._victim_file.write(f"Longitude: {strongest.longitude:.6f}\n")
                    self._victim_file.write(f"RSSI: {strongest.rssi:.1f} dBm\n")
                
                # Log summary
                summary = f"Mission summary:\n"
                summary += f"  Total data points: {len(self._collection)}\n"
                summary += f"  Detected {len(victims)} victim(s):\n"
                for i, (v_lat, v_lon, v_rssi) in enumerate(victims, 1):
                    summary += f"    Victim {i}: ({v_lat:.6f}, {v_lon:.6f}), RSSI: {v_rssi:.1f} dBm\n"
                logger.info(summary)
            else:
                # Fallback to strongest point if algorithm finds no victims
                strongest = self._collection.strongest()
                if strongest:
                    self._victim_file.write("No victims detected by algorithm.\n")
                    self._victim_file.write(f"\nStrongest single point:\n")
                    self._victim_file.write(f"Latitude: {strongest.latitude:.6f}\n")
                    self._victim_file.write(f"Longitude: {strongest.longitude:.6f}\n")
                    self._victim_file.write(f"RSSI: {strongest.rssi:.1f} dBm\n")
                    
                    # Send strongest point as fallback
                    self._mav._send_tunnel_message(
                        strongest.latitude,
                        strongest.longitude,
                        strongest.rssi,
                        TYPE_VICTIM_MARKER
                    )
                else:
                    self._victim_file.write("No data points collected during mission.\n")
                    logger.info(f"No data points collected\n")
                    print("[CONTROL] Mission complete: No data points collected")
        else:
            self._victim_file.write("No data points collected during mission.\n")
            logger.info(f"No data points collected\n")
            print("[CONTROL] Mission complete: No data points collected")

        # close files 
        self._data_file.close() 
        self._victim_file.close()

        self._global_position_file.close() 
        self._local_position_file.close()
        self._wifi_rssi_file.close() 

        logger.info(f"Mission files closed for {self._mission_dir}")
