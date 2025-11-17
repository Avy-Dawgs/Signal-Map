'''
For the monitoring of WiFi RSSI.
'''
from threading import Thread
from typing import Optional
from pyric import pyw 
from scapy.all import Packet, sniff
from scapy.layers.dot11 import RadioTap
from subprocess import call
from time import time
from numpy import argmax, array
from shared_types import RssiTime
import logging


class WifiInterface:
    '''
    Class to interface with and control a WiFi card.
    '''
    _card_name: str
    __card: pyw.Card
    _channel: int

    def __init__(
            self, 
            card_name: str
            ) -> None: 
        '''
        Contructor.

        args: 
            card_name: name of wireless card
        '''
        cards = pyw.winterfaces() 

        if card_name in cards: 
            self._card_name = card_name
        elif f"{card_name}mon" in cards:
            self._card_name = f"{card_name}mon"
        else:
            raise Exception("Given card not found.")

        self.__card = pyw.getcard(self._card_name)

    def start_monitor_mode(
            self, 
            kill_processes: bool
            ) -> None : 
        '''
        Enter monitor mode, calling a callback 
        every time theres a new RSSI value.

        args: 
            kill_processes: kill conflicting processes
        '''
        if kill_processes:
            logging.info("Killing processes that conflict with monitor mode.")
            call(["airmon-ng", "check", "kill"]) 

        if not self._card_name.endswith("mon"):
            logging.info("Starting monitor mode.")

            # this is the best way i've found to enter monitor mode
            call(["airmon-ng", "start", self._card_name])
            self._card_name += "mon"
            self.__card = pyw.getcard(self._card_name)
            logging.info("Monitor mode started.")
        else: 
            logging.info("Monitor mode was already started.")

    def stop_monitor_mode(
            self, 
            start_network_manager: bool
            ) -> None: 
        '''
        Stop monitor mode.

        args: 
            start_network_manager: start network manager (to reconnect to networks)
        '''
        logging.info("Stopping monitor mode.")
        call(["airmon-ng", "stop", self._card_name])
        logging.info("Monitor mode stopped.")

        if start_network_manager:
            logging.info("Staring NetworkManager.")
            call(["systemctl", "start", "NetworkManager"])

    def set_channel(
            self, 
            channel: int
            ) -> None: 
        '''
        Set the channel of the WiFi card.
        '''
        pyw.chset(self.__card, channel)
        self._channel = channel
        logging.info(f"WiFi channel set to {channel}.")


class WifiRssiMonitor: 
    '''
    Class to monitor WiFi packets for Rssi.
    '''
    __wifi: WifiInterface
    _rssi_list: list[float]
    _time_list: list[float]
    __monitor_thread: Thread
    _seconds_of_data: int

    def __init__(
            self, 
            wifi: WifiInterface, 
            ch: int, 
            kill_processes: bool,
            ) -> None: 
        '''
        Contructor. Starts monitor on given channel.
        '''
        self.__wifi = wifi
        self.__wifi.start_monitor_mode(kill_processes)
        self.__wifi.set_channel(ch)
        self._rssi_list = []
        self._time_list = []
        self._seconds_of_data = 2

        logging.info("Staring packet capture.")
        self.__monitor_thread = Thread(target=self.__monitor, daemon=True)
        self.__monitor_thread.start()

    def __monitor(
            self
            ) -> None: 
        '''
        Run as background thread to monitor rssi.
        '''
        sniff(iface=self.__wifi._card_name, prn=self.__handle_packet, store=0)

    def __handle_packet(
            self, 
            pkt: Packet
            ) -> None: 
        '''
        Handle a packet.
        '''
        try: 
            radiotap = pkt.getlayer(RadioTap)
            if radiotap:
                rssi = float(radiotap.dBm_AntSignal)
                self._rssi_list.append(rssi) 
                self._time_list.append(float(pkt.time))
        except AttributeError: 
            pass

    def get_max_rssi(
            self, 
            n: float
            ) -> Optional[RssiTime]:
        '''
        Get the max value over the last n seconds.

        args: 
            n: seconds to consider for max 

        returns: RssiTime 
        '''
        current_time = time()

        # remove elements that are too old
        nsec_ago = current_time - self._seconds_of_data
        elements_to_delete = 0
        for t in self._time_list:
            if (t < nsec_ago):
                elements_to_delete += 1
            else: 
                break
        del self._rssi_list[:elements_to_delete]
        del self._time_list[:elements_to_delete]

        # get max over last n seconds
        nsec_ago = current_time - n
        last_sec_times = [x for x in self._time_list if x > nsec_ago]
        last_sec_rssi = self._rssi_list[len(self._rssi_list) - len(last_sec_times) :]

        if len(last_sec_rssi) == 0: 
            logging.info(f"No rssi data for last {n} seconds")
            return None

        idx = argmax(array(last_sec_rssi))
        return RssiTime(last_sec_rssi[idx], last_sec_times[idx])

    def stop(
            self
            ) -> None:
        '''
        Stop monitoring.
        '''
        logging.info("Stopping packet capture.") 
        self.__wifi.stop_monitor_mode(False)
