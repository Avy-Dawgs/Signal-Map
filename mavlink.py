'''
For communicating with the drone/base station over MavLink.
'''
import csv
import _csv
from io import TextIOWrapper
from pymavlink import mavutil
from enum import Enum
import struct
from pymavlink.dialects.v20 import ardupilotmega as dialect
from typing import Callable, Optional
from time import time, sleep
from threading import Thread
import logging


logger = logging.getLogger(__name__) 
logger.level = logging.DEBUG


# MAVLink TUNNEL constants
PAYLOAD_LEN = 12
PAYLOAD_BUF = 128
TYPE_RSSI_GLOBAL = 4  # Tunnel type for lat/lon/rssi heatmap data
TYPE_VICTIM_MARKER = 2  # Tunnel type for final victim marker


def pack_triplet(lat: float, lon: float, rssi: float) -> bytes:
    """Pack lat/lon/rssi into tunnel payload format (12 bytes data + padding)."""
    body = struct.pack("<fff", float(lat), float(lon), float(rssi))
    return body + bytes(PAYLOAD_BUF - len(body))


class MissionState(Enum): 
    INACTIVE = 0 
    ACTIVE = 1 
    RETURNING = 2


class MavlinkConnectionManager: 
    '''
    Monitors mavlink connection for required telemetry data, 
    can send tunnel messages.
    '''
    _mav_conn: mavutil.mavfile
    new_global_position_cb: Optional[Callable[[float, float, float], None]]
    new_local_position_cb: Optional[Callable[[float, float, float], None]]
    mission_started_cb: Optional[Callable[[], None]]
    mission_ended_cb: Optional[Callable[[], None]]
    _base_station_system_id: int
    _drone_system_id: int 
    _flight_controller_component_id: int
    _raspberrypi_component_id: int
    __telemetry_thread: Thread
    __time_last_heartbeat_sent: float
    __heartbeat_period: float = 1.0     # this could become parameter
    _mission_state: MissionState

    def __init__(
            self, 
            mav_str: str,
            drone_system_id: int, 
            flight_controller_component_id: int,
            raspberrypi_component_id: int,
            base_station_system_id: int,
            baud: int
            ) -> None: 
        '''
        Constructor. 
        Starts monitoring.
        '''

        self.mission_ended_cb = None 
        self.mission_started_cb = None
        self.new_global_position_cb = None 
        self.new_local_position_cb = None

        self._mission_state = MissionState.INACTIVE
        self.__time_last_heartbeat_sent = 0.0

        # retry infinitely
        connected = False
        while not connected:
            try:
                self._mav_conn = mavutil.mavlink_connection(
                        mav_str, 
                        source_system=drone_system_id, 
                        source_component=raspberrypi_component_id,
                        baud=baud
                    )
                connected = True
            except: 
                logger.critical("MavLink connection failed, will retry infinitely.")
                sleep(0.5)
                pass

        self._drone_system_id = drone_system_id 
        self._flight_controller_component_id = flight_controller_component_id
        self._raspberrypi_component_id = raspberrypi_component_id
        self._base_station_system_id = base_station_system_id

        self.__telemetry_thread = Thread(
                target=self.__telemetry_loop, 
                daemon=True
                )
        self.__telemetry_thread.start()

    def __send_heartbeat(
            self
            ) -> None: 
        '''
        Send a heartbeat.
        '''
        self._mav_conn.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,    # type
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,          # autopilot
            0,                                              # base mode, 
            0,                                              # custom mode
            0                                               # system status
        )
        self.__time_last_heartbeat_sent = time()

    def __set_message_interval(
            self, 
            msg_id, 
            interval: float
            ) -> None: 
        '''
        Set message interval for given message id.

        args: 
            msg_id: id of message to set interval for 
            interval: message interval given as in seconds
        '''
        self._mav_conn.mav.command_long_send(
            self._drone_system_id,  # target sys
            self._flight_controller_component_id,  # target comp
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msg_id,
            int(interval*1e6),
            0, 0, 0, 0, 0
            )

    def __telemetry_loop(
            self
            ) -> None: 
        '''
        Listen for telemetry messages and set off callbacks.
        '''
        # wait for and send heartbeat
        self._mav_conn.wait_heartbeat()
        self.__send_heartbeat()

        # setup intervals
        self.__set_message_interval(            
                mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT,       # prob unnessessary for heartbeat
                1
                )
        self.__set_message_interval(
                mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 
                0.25                # a bit slower than the default ardupilot gps update rate of 0.2 (to prevent getting the same reading twice)
                )

        self.__set_message_interval(
                mavutil.mavlink.MAVLINK_MSG_ID_MISSION_CURRENT, 
                1
                )

        self.__set_message_interval(
                mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 
                0.25
                )

        while True: 
            # send heartbeat
            if (time() - self.__time_last_heartbeat_sent) >= self.__heartbeat_period:
                self.__send_heartbeat()
            
            msg = self._mav_conn.recv_match(
                    type = [
                        "GLOBAL_POSITION_INT", 
                        "HEARTBEAT", 
                        "MISSION_CURRENT", 
                        "LOCAL_POSITION_NED",
                        ], 
                    blocking=True, 
                    timeout=0.15
                    )

            if not msg: 
                continue

            match msg.get_type(): 
                case "LOCAL_POSITION_NED":
                    self.__local_position_ned_handler(msg)
                case "GLOBAL_POSITION_INT": 
                    self.__global_position_int_handler(msg)
                case "HEARTBEAT": 
                    self.__heartbeat_handler(msg)
                case "MISSION_CURRENT":
                    self.__mission_current_handler(msg)


    def __mission_current_handler(
            self,
            msg: dialect.MAVLink_mission_current_message
            ) -> None: 
        '''
        Handles a mission current message.
        '''
        if msg.get_srcSystem() != self._drone_system_id:
            return 

        match self._mission_state: 
            # transition when mission goes active
            case MissionState.INACTIVE: 
                if msg.mission_state == dialect.MISSION_STATE_ACTIVE: 
                    logger.info("Mission state transition from INACTIVE to ACTIVE.")
                    self._mission_state = MissionState.ACTIVE 
                    if self.mission_started_cb: 
                        self.mission_started_cb()
            # transition when last waypoint reached
            case MissionState.ACTIVE: 
                if msg.seq == msg.total:
                    logger.info("Mission state transition from ACTIVE to RETURNING.")
                    self._mission_state = MissionState.RETURNING
                    # prob should rename this cb
                    if self.mission_ended_cb: 
                        self.mission_ended_cb()
            # transiiton when drone lands
            case MissionState.RETURNING:
                if msg.mission_state == dialect.MISSION_STATE_COMPLETE: 
                    logger.info("Mission state transition from RETURNING to INACTIVE.")
                    self._mission_state = MissionState.INACTIVE

    def __heartbeat_handler(
            self, 
            msg: dialect.MAVLink_heartbeat_message
            ) -> None: 
        '''
        Handles a heartbeat messae.
        '''
        # reject messages that don't come from the drone's flight controller
        if msg.get_srcSystem() != self._drone_system_id:
            return

        if self._mission_state == MissionState.ACTIVE: 
            if msg.custom_mode == dialect.COPTER_MODE_RTL: 
                logger.info("Detected RTL, ending mission here.")
                self._mission_state = MissionState.INACTIVE 
                if self.mission_ended_cb: 
                    self.mission_ended_cb()

    def __local_position_ned_handler(
            self, 
            msg: dialect.MAVLink_local_position_ned_message,
            ) -> None:
        '''
        Hanndles a local position message.
        '''
        if msg.get_srcSystem() != self._drone_system_id:
            return 

        if self.new_local_position_cb: 
            self.new_local_position_cb(
                    msg.x, 
                    msg.y, 
                    self._mav_conn.messages["LOCAL_POSITION_NED"]._timestamp
                    )

    def __global_position_int_handler(
            self, 
            msg: dialect.MAVLink_global_position_int_message
            ) -> None: 
        '''
        Handles a global position int message.
        '''
        if msg.get_srcSystem() != self._drone_system_id:
            return 

        if self.new_global_position_cb:
            self.new_global_position_cb(
                    float(msg.lat)/1e7, 
                    float(msg.lon)/1e7, 
                    self._mav_conn.messages["GLOBAL_POSITION_INT"]._timestamp
                    )

    def _send_tunnel_message(
            self, 
            lat: float, 
            lon: float, 
            rssi: float, 
            msg_type: int = TYPE_RSSI_GLOBAL
            ) -> None:
        """
        Send a tunnel message with lat/lon/rssi data.
        """
        payload = pack_triplet(lat, lon, rssi)
        self._mav_conn.mav.tunnel_send(
            target_system=self._base_station_system_id,
            target_component=0,             # doesn't matter
            payload_type=msg_type,
            payload_length=PAYLOAD_LEN,
            payload=payload
        )
        type_name = "HEATMAP" if msg_type == TYPE_RSSI_GLOBAL else "MARKER"
        logger.info(f"[TX] TUNNEL type={msg_type} ({type_name}) lat={lat:.6f} lon={lon:.6f} rssi={rssi:.1f}")


class MavlinkTelemetryMonitor: 
    '''
    Monitors data coming from mavlink.
    '''
    _lat_list: list[float]
    _lon_list: list[float]
    _global_time_list: list[float]

    _x_list: list[float] 
    _y_list: list[float]
    _local_time_list: list[float]

    _mav: MavlinkConnectionManager

    __global_position_csv_writer: _csv.writer
    __local_position_csv_writer: _csv.writer
    __log_incoming: bool 
    '''Log the data as it comes in?'''

    def __init__(
            self, 
            mav: MavlinkConnectionManager
            ) -> None: 
        '''
        Contructor.
        '''
        self._mav = mav

        self.__log_incoming = False

        self._lat_list = [] 
        self._lon_list = [] 
        self._global_time_list = []

        self._x_list = [] 
        self._y_list = [] 
        self._local_time_list = []

        # register for new position events
        self._mav.new_global_position_cb = self.__new_global_position
        self._mav.new_local_position_cb = self.__new_local_position

    def __new_global_position(
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
        self._global_time_list.append(time)

        if self.__log_incoming: 
            self.__global_position_csv_writer.writerows([[lat, lon, time]])

    def __new_local_position(
            self, 
            x: float, 
            y: float, 
            time: float
            ) -> None:
        '''
        Save a new local position.
        '''
        self._x_list.append(x) 
        self._y_list.append(y) 
        self._local_time_list.append(time)

        if self.__log_incoming: 
            self.__local_position_csv_writer.writerows([[x, y, time]])

    def start_logging(
            self,
            local_position_file: TextIOWrapper,
            global_position_file: TextIOWrapper,
            ) -> None: 
        '''
        Start logging incoming data.
        '''
        self.__log_incoming = True 

        self.__global_position_csv_writer = csv.writer(global_position_file)
        self.__local_position_csv_writer = csv.writer(local_position_file)

    def stop_logging(
            self, 
            ) -> None: 
        '''
        Stop logging incoming data. 
        '''
        self.__log_incoming = False

    def write_local_positions_to_file(
            self, 
            file: TextIOWrapper,
            ) -> None: 
        '''
        Write local positions to a file.
        '''
        w = csv.writer(file)
        w.writerows(zip(self._x_list, self._y_list, self._local_time_list))

    def write_global_positions_to_file(
            self, 
            file: TextIOWrapper, 
            ) -> None: 
        '''
        Write global positions to a file.
        '''
        w = csv.writer(file) 
        w.writerows(zip(self._lat_list, self._lon_list, self._global_time_list))

    def clear(
            self
            ) -> None: 
        '''
        Clear the list. (will leave two list elements)
        '''
        n = len(self._lat_list)      # note that lat and lon lists are same len
        del self._lat_list[:n-2]
        del self._lon_list[:n-2] 
        del self._global_time_list[:n-2]

        n = len(self._x_list) 
        del self._x_list[:n-2]
        del self._y_list[:n-2]
        del self._local_time_list[:n-2]
