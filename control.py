'''
Central logic for the daemon.
'''
from io import TextIOWrapper
from mavlink import MavlinkConnectionManager, MavlinkTelemetryMonitor, TYPE_RSSI_GLOBAL, TYPE_VICTIM_MARKER
from wifi_rssi import WifiRssiMonitor
from DataPoint import DataPoint, DataPointCollection
from processing import location_interpolate
from time import sleep, time
from os import makedirs, path
from datetime import datetime
import struct

class Control: 
    '''
    Overall program logic.
    '''
    _mav: MavlinkConnectionManager 
    _telem: MavlinkTelemetryMonitor
    _wifi_mon: WifiRssiMonitor
    _log_dir: str
    '''directory to store files'''
    _message_file: TextIOWrapper
    '''general log file'''
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
            while True: 
                if self.__mission_started_flag: 
                    self.__mission_started_flag = False
                    break
                sleep(0.25)

            # progress through the states
            self.__mision_init()
            self.__run_mission()
            self.__mission_deinit()

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
            print(f"[WARNING] Could not write mission counter file, using counter={counter}")
        
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
        self._message_file = open(f"{self._mission_dir}/messages.log", "w")
        self._data_file = open(f"{self._mission_dir}/signal_map.dat", "wb")
        self._victim_file = open(f"{self._mission_dir}/victim.txt", "w")
        
        # Clear collection for new mission
        self._collection.clear()
        self._last_processed_rssi_time = 0.0  # Reset for new mission
        
        # Log mission start
        self._message_file.write(f"[{datetime.now().isoformat()}] Mission #{mission_num} started\n")
        self._message_file.flush()
        print(f"[CONTROL] Mission #{mission_num} started, logging to {self._mission_dir}")

    def __run_mission(
            self
            ) -> None:
        '''
        Runs during the main mission.
        Interpolates location, creates data points, sends tunnel messages, and logs data.
        '''
        rssi_check_window = 0.2  # seconds: short window to detect new pulses
        
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
                                self._telem._time_list,
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
                            log_msg = f"[{datetime.now().isoformat()}] DataPoint (beacon pulse): lat={point.latitude:.6f}, lon={point.longitude:.6f}, rssi={point.rssi:.1f} dBm\n"
                            self._message_file.write(log_msg)
                            self._message_file.flush()
                            
                            print(f"[CONTROL] Processed beacon pulse: rssi={point.rssi:.1f} dBm at ({point.latitude:.6f}, {point.longitude:.6f})")
                            
                        except Exception as e:
                            error_msg = f"[{datetime.now().isoformat()}] Error processing data point: {e}\n"
                            self._message_file.write(error_msg)
                            self._message_file.flush()
                            print(f"[CONTROL] Error: {e}")
                    else:
                        # Not enough location data yet - skip this pulse
                        pass
            
            # Small sleep to prevent waiting while still being responsive to pulses
            sleep(0.1) 


    def __mission_deinit(
            self
            ) -> None: 
        '''
        Run once when mission ends.
        Calculates victim location, sends final marker, and writes summary files.
        '''
        self._message_file.write(f"[{datetime.now().isoformat()}] Mission ended\n")
        
        # Calculate victim location using weighted average of strongest signals
        if len(self._collection) > 0:
            # Get top 10% of strongest signals (minimum 3 points)
            all_points = sorted(self._collection.data_points, key=lambda p: p.rssi, reverse=True)
            top_count = max(3, len(all_points) // 10)
            top_points = all_points[:top_count]
            
            if len(top_points) > 0:
                # Weighted average by RSSI (convert to linear scale for weighting)
                total_weight = 0.0
                weighted_lat = 0.0
                weighted_lon = 0.0
                rssi_sum = 0.0
                
                for point in top_points:
                    # Convert dBm to linear scale: weight = 10^(rssi/10)
                    weight = 10 ** (point.rssi / 10.0)
                    total_weight += weight
                    weighted_lat += point.latitude * weight
                    weighted_lon += point.longitude * weight
                    rssi_sum += point.rssi
                
                if total_weight > 0:
                    victim_lat = weighted_lat / total_weight
                    victim_lon = weighted_lon / total_weight
                    avg_rssi = rssi_sum / len(top_points)
                    
                    # Send victim marker
                    self._mav._send_tunnel_message(
                        victim_lat,
                        victim_lon,
                        avg_rssi,
                        TYPE_VICTIM_MARKER
                    )
                    
                    # Write victim location to file
                    self._victim_file.write(f"Victim Location (weighted average of top {len(top_points)} signals):\n")
                    self._victim_file.write(f"Latitude: {victim_lat:.6f}\n")
                    self._victim_file.write(f"Longitude: {victim_lon:.6f}\n")
                    self._victim_file.write(f"Average RSSI: {avg_rssi:.1f} dBm\n")
                    self._victim_file.write(f"\nStrongest single point:\n")
                    strongest = self._collection.strongest()
                    if strongest:
                        self._victim_file.write(f"Latitude: {strongest.latitude:.6f}\n")
                        self._victim_file.write(f"Longitude: {strongest.longitude:.6f}\n")
                        self._victim_file.write(f"RSSI: {strongest.rssi:.1f} dBm\n")
                        # Calculate distance between methods
                        dist = strongest.distance_to(DataPoint(victim_lat, victim_lon, avg_rssi))
                        self._victim_file.write(f"\nDistance between methods: {dist:.2f} meters\n")
                    
                    # Log summary
                    summary = f"[{datetime.now().isoformat()}] Mission summary:\n"
                    summary += f"  Total data points: {len(self._collection)}\n"
                    summary += f"  Victim location: ({victim_lat:.6f}, {victim_lon:.6f})\n"
                    summary += f"  Average RSSI: {avg_rssi:.1f} dBm\n"
                    self._message_file.write(summary)
                    self._message_file.flush()
                    print(f"[CONTROL] Mission complete: {len(self._collection)} data points collected")
                    print(f"[CONTROL] Victim location: ({victim_lat:.6f}, {victim_lon:.6f})")
        else:
            self._victim_file.write("No data points collected during mission.\n")
            self._message_file.write(f"[{datetime.now().isoformat()}] No data points collected\n")
            self._message_file.flush()
            print("[CONTROL] Mission complete: No data points collected")

        # close files 
        self._message_file.close() 
        self._data_file.close() 
        self._victim_file.close()
        print(f"[CONTROL] Log files closed for {self._mission_dir}")
