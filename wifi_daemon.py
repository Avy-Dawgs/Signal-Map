'''
Daemon to run on comanion computer and collect wifi signals.
'''
from typing import Optional
from sys import argv
from xmltodict import parse, unparse
from xml.dom.minidom import parseString

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

    # init mavlink manager and telemetry
    mavlink_manager = MavlinkConnectionManager(
            params.mavlink_connection_string, 
            params.drone_system_id, 
            params.raspberry_pi_component_id, 
            params.base_station_system_id
            )
    mavlink_telemetry = MavlinkTelemetryMonitor(
            mavlink_manager
            )

    # launch wifi sniffing 
    wifi_interface = WifiInterface(params.wifi_card_name)
    wifi_rssi_monitor = WifiRssiMonitor(
            wifi_interface, 
            params.wifi_channel, 
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
    drone_component_id: int
    raspberry_pi_component_id: int
    base_station_system_id: int

    wifi_card_name: str 
    wifi_channel: int

    beacon_period: float 

    log_directory: str

    def __init__(self, filename: Optional[str]) -> None:
        '''
        Contructor. 
        Load from a file if given.

        args: 
            filename: name of file to load from
        '''
        if not filename: 
            return

        with open(filename, "r") as f: 
            d = parse(f.read())

        for key, value in d.items():
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
