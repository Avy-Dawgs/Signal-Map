'''
For communicating with the drone/base station over MavLink.
'''
from pymavlink import mavutil
from typing import Callable
from time import time
from threading import Thread


class MavlinkConnectionManager: 
    '''
    Monitors mavlink connection for required telemetry data, 
    can send tunnel messages.
    '''
    _mav: mavutil.mavfile
    __new_position_cb: Callable[[float, float, float], None]
    __mission_started_cb: Callable[[], None]
    __mission_ended_cb: Callable[[], None]
    _base_station_system_id: int
    __telemetry_thread: Thread

    def __init__(
            self, 
            mav: mavutil.mavfile, 
            base_station_system_id: int,
            new_position_cb: Callable[[float, float, float], None], 
            mission_started_cb: Callable[[], None], 
            mission_ended_cb: Callable[[], None]
            ) -> None: 
        '''
        Constructor. 
        Starts monitoring.
        '''

        # TODO setup message latencies?
        # TODO send heartbeat
        self._base_station_system_id = base_station_system_id
        self._mav = mav
        self.__new_position_cb = new_position_cb 
        self.__mission_started_cb = mission_started_cb 
        self.__mission_ended_cb = mission_ended_cb

        self.__telemetry_thread = Thread(target=self.__telemetry_loop, daemon=True)
        self.__telemetry_thread.start()

    def __telemetry_loop(self): 
        '''
        Listen for telemetry messages and set off callbacks.
        '''
        while True: 
            # TODO listen for mission start and end
            msg = self._mav.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=0.25)

            if msg: 
                time_of_message = time() - self._mav.time_since("GLOBAL_POSITION_INT")
                self.__new_position_cb(float(msg.lat)/1e7, float(msg.lon)/1e7, time_of_message)


class MavlinkTelemetryData: 
    '''
    Holds data coming from Mavlink telemetry.
    '''
    _lat_list: list[float]
    _lon_list: list[float]
    _time_list: list[float]

    def __init__(
            self, 
            ) -> None: 
        '''
        Contructor.
        '''
        self._lat_list = [] 
        self._lon_list = [] 
        self._time_list = []

    def new_global_position(
            self, 
            lat: float, 
            lon: float, 
            time: float
            ) -> None:
        '''
        Track a new global position.
        '''
        self._lat_list.append(lat) 
        self._lon_list.append(lon) 
        self._time_list.append(time)

    def clear(
            self
            ) -> None: 
        '''
        Clear the list (will leave two list elements).
        '''
        n = len(self._lat_list) # all lists same length
        del self._lat_list[:n-2]
        del self._lon_list[:n-2] 
        del self._time_list[:n-2]
