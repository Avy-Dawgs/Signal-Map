'''
Daemon to run on comanion computer and collect wifi signals.
'''
from typing import Optional
from sys import argv
from xmltodict import parse, unparse
from xml.dom.minidom import parseString
import logging
from os import path, makedirs

from mavlink import MavlinkConnectionManager, MavlinkTelemetryMonitor
from control import Control
from wifi_rssi import WifiInterface, WifiRssiMonitor


'''
Software layout: (classes)
* WifiInterface 
    * set up wifi card by entering monitor mode 
    * change current channel
    * exit monitor mode
* WifiRssiMonitor 
    * sniffs wifi packets for rssi values 
    * records time as rssi values come in
    * potentially channel hops if no/few packets are being received
* MavlinkConnectionManager
    * listens for telemetry data, setting off events/callbacks for certain messages
        * global location values
        * potentially velocity values
        * start of mission 
        * end of mission
    * send tunnel messages 
    * send heartbeat (every second)
* MavlinkTelemetryMonitor 
    * collect new location data and track it historically
    * potentially collect velocity data and track it historically
* DataPointCollection (model)
    * model of rssi values at positions in space
* Control
    * determines period of new wifi data to simulate avalanche beacon period 
    * handles the processing of rssi data to determine location that they were received at 
    * starts and stops depending on mission status (start/end)
    * send data back to flight controller
'''


def main(argv: list[str]): 
    '''
    Entry Point.
    '''
    if len(argv) != 2: 
        print("Please provide the parameter file as the only argument.")
        return

    # load params 
    params = DaemonParams(argv[1])

    makedirs(params.log_directory, exist_ok=True)

    streamhandler = logging.StreamHandler() 
    filehandler = logging.FileHandler(path.join(params.log_directory, "wifi_daemon.log"))
    streamhandler.setFormatter(logging.Formatter("[%(name)s %(levelname)s]: %(message)s"))
    filehandler.setFormatter(logging.Formatter("[%(name)s %(levelname)s %(asctime)s]: %(message)s"))
    logger = logging.getLogger()
    logger.handlers = [
            streamhandler,
            filehandler,
            ] 
    logger.level = logging.INFO 

    counter_file = path.join(params.log_directory, ".daemon_run_count")
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
        with open(counter_file, "w") as f:
            f.write(str(counter))
    except IOError:
        # If we can't write, still use the counter but warn
        logger.warning(f" Could not write to run count file")

    logger.info("=================================================")
    logger.info(f"This daemon has been started {counter} times.")
    logger.info("=================================================")


    # init mavlink manager and telemetry
    mavlink_manager = MavlinkConnectionManager(
            params.mavlink_connection_string, 
            params.drone_system_id, 
            params.flight_controller_component_id,
            params.raspberry_pi_component_id, 
            params.base_station_system_id, 
            params.serial_baud
            )
    mavlink_telemetry = MavlinkTelemetryMonitor(
            mavlink_manager
            )

    # launch wifi sniffing 
    wifi_interface = WifiInterface(params.wifi_card_name)
    wifi_rssi_monitor = WifiRssiMonitor(
            wifi_interface, 
            params.beacon_ssid,
            True        # kill processes
            )

    # init controller 
    ctrl = Control(
            mavlink_manager, 
            mavlink_telemetry, 
            wifi_rssi_monitor, 
            params.log_directory
            )

    # run it!!
    ctrl.run()


class DaemonParams: 
    '''
    Parameters for this daemon.
    '''
    mavlink_connection_string: str 
    drone_system_id: int
    flight_controller_component_id: int
    raspberry_pi_component_id: int
    base_station_system_id: int

    wifi_card_name: str 

    beacon_period: float 
    beacon_ssid: str

    log_directory: str

    serial_baud: int

    def __init__(self, filename: Optional[str]) -> None:
        '''
        Constructor. 
        Load from a file if given.

        args: 
            filename: name of file to load from
        '''
        if not filename: 
            return

        with open(filename, "r") as f: 
            d = parse(f.read())

        if "DaemonParams" in d:
            params_dict = d["DaemonParams"]
            for key, value in params_dict.items():
                # Convert string values to appropriate types
                if key in ["drone_system_id", "flight_controller_component_id", "raspberry_pi_component_id", 
                          "base_station_system_id", "serial_baud"]:
                    value = int(value)
                elif key == "beacon_period":
                    value = float(value)
                setattr(self, key, value)

    def save(self, filename: str) -> None: 
        '''
        Save to a file.

        args: 
            filename: name of file to save to
        '''
        d = {"DaemonParams" : vars(self)}
        xml_str = parseString(unparse(d)).toprettyxml()

        with open(filename, "w") as f: 
            f.write(xml_str)

    def __str__(self) -> str: 
        '''
        Convert to string.

        returns: a dictionary representation of this object as a string
        '''
        return str(vars(self))


if __name__ == "__main__": 
    main(argv)
