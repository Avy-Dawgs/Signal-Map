'''
Central logic for the daemon.
'''
from io import TextIOWrapper
from mavlink import MavlinkConnectionManager, MavlinkTelemetryMonitor
from wifi_rssi import WifiRssiMonitor
from time import sleep
from os import makedirs
from datetime import datetime

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
        Contructor.
        '''
        self._mav = mav_manager 
        self._telem = mav_telemetry
        self._wifi_mon = wifi_monitor
        self._log_dir = log_directory

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
        Handle misison started event.
        '''
        self.__mission_started_flag = True

    def __mission_ended_handler(
            self
            ) -> None: 
        '''
        Handle mission ended event.
        '''
        self.__mission_ended_flag = True

    def __mision_init(
            self
            ) -> None:
        '''
        Run once when mission starts.
        '''
        # TODO prob need different approach for naming here because rpi has no rtc (time will reset on boot)

        # create directory / files for logging
        d = self._log_dir + "/" + str(datetime.now())
        makedirs(d)
        self._message_file = open(f"{d}/messages.log")
        self._data_file = open(f"{d}/signal_map.dat")
        self._victim_file = open(f"{d}/victim.txt")

    def __run_mission(
            self
            ) -> None:
        '''
        Runs during the main mission.
        '''

        while True:
            # check for end of mission.
            if self.__mission_ended_flag:
                self.__mission_ended_flag = False 
                break 

            # TODO 


    def __mission_deinit(
            self
            ) -> None: 
        '''
        Run once when mission ends.
        '''
        # TODO anything else to do here (maybe do a final analysis)

        # close files 
        self._message_file.close() 
        self._data_file.close() 
        self._victim_file.close()
