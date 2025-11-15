'''
For communicating with the drone/base station over MavLink.
'''
from pymavlink import mavutil
import struct
from pymavlink.dialects.v20 import ardupilotmega as dialect
from typing import Callable
from time import time
from threading import Thread


# MAVLink TUNNEL constants
PAYLOAD_LEN = 12
PAYLOAD_BUF = 128
TYPE_RSSI_GLOBAL = 4  # Tunnel type for lat/lon/rssi heatmap data
TYPE_VICTIM_MARKER = 2  # Tunnel type for final victim marker


def pack_triplet(lat: float, lon: float, rssi: float) -> bytes:
    """Pack lat/lon/rssi into tunnel payload format (12 bytes data + padding)."""
    body = struct.pack("<fff", float(lat), float(lon), float(rssi))
    return body + bytes(PAYLOAD_BUF - len(body))

class MavlinkConnectionManager: 
    '''
    Monitors mavlink connection for required telemetry data, 
    can send tunnel messages.
    '''
    _mav_conn: mavutil.mavfile
    new_position_cb: Callable[[float, float, float], None]
    mission_started_cb: Callable[[], None]
    mission_ended_cb: Callable[[], None]
    _base_station_system_id: int
    _drone_system_id: int 
    _raspberrypi_component_id: int
    _enc: dialect.MAVLink
    __telemetry_thread: Thread
    __mission_active: bool
    __time_last_heartbeat_sent: float
    __heartbeat_period: float = 1.0     # this could become parameter

    def __init__(
            self, 
            mav_str: str,
            drone_system_id: int, 
            raspberrypi_component_id: int,
            base_station_system_id: int,
            baud: int
            ) -> None: 
        '''
        Constructor. 
        Starts monitoring.
        '''

        self.__mission_active = False
        self.__time_last_heartbeat_sent = 0.0

        self._mav_conn = mavutil.mavlink_connection(
                mav_str, 
                source_system=drone_system_id, 
                source_component=raspberrypi_component_id,
                baud=baud
            )

        # encoder
        self._enc = dialect.MAVLink(
                None, 
                drone_system_id, 
                raspberrypi_component_id
                )

        # TODO setup message intervals
        #   - 0.1 seconds location 
        #   - 1 seconds mission done
        # TODO handle heartbeats somehow (probably send and receive)
        self._drone_system_id = drone_system_id 
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
        self._mav_conn.mav.message_interval_send(
                msg_id,         # message id 
                int(interval*1e6)  # interval 
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
                0.1
                )

        while True: 
            # send heartbeat
            if time() - self.__time_last_heartbeat_sent >= self.__heartbeat_period: 
                self.__send_heartbeat()
            
            msg = self._mav_conn.recv_match(
                    type=["GLOBAL_POSITION_INT", "HEARTBEAT"], 
                    blocking=True, 
                    timeout=0.15
                    )

            if not msg: 
                continue

            match msg.get_type(): 
                case "GLOBAL_POSITION_INT": 
                    self.__global_position_int_handler(msg)
                case "HEARTBEAT": 
                    self.__heartbeat_handler(msg)

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

        # mision is active, evaulate a transition to inactive
        if self.__mission_active: 
            if msg.custom_mode != dialect.COPTER_MODE_AUTO: 
                self.__mission_active = False
                if self.mission_ended_cb: 
                    self.mission_ended_cb()
        # misison is not active, evaulate a transition to active
        else: 
            if msg.custom_mode == dialect.COPTER_MODE_AUTO:
                self.__mission_active = True 
                if self.mission_started_cb: 
                    self.mission_started_cb()

    def __global_position_int_handler(
            self, 
            msg: dialect.MAVLink_global_position_int_message
            ) -> None: 
        '''
        Handles a global position int message.
        '''
        if self.new_position_cb:
            self.new_position_cb(
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
        print(f"[TX] TUNNEL type={msg_type} ({type_name}) lat={lat:.6f} lon={lon:.6f} rssi={rssi:.1f}")


class MavlinkTelemetryMonitor: 
    '''
    Monitors data coming from mavlink.
    '''
    _lat_list: list[float]
    _lon_list: list[float]
    _time_list: list[float]

    _mav: MavlinkConnectionManager

    def __init__(
            self, 
            mav: MavlinkConnectionManager
            ) -> None: 
        '''
        Contructor.
        '''
        self._mav = mav

        self._lat_list = [] 
        self._lon_list = [] 
        self._time_list = []

        # register for new global position event
        self._mav.new_position_cb = self.__new_global_position

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
        self._time_list.append(time)

    def clear(
            self
            ) -> None: 
        '''
        Clear the list. (will leave two list elements)
        '''
        n = len(self._lat_list)      # note that all lists are same length
        del self._lat_list[:n-2]
        del self._lon_list[:n-2] 
        del self._time_list[:n-2]
