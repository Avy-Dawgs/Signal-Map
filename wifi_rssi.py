'''
For the monitoring of WiFi RSSI.
'''
from threading import Thread, current_thread
from typing import Optional
from pyric import pyw 
from scapy.all import Packet, sniff
from scapy.layers.dot11 import RadioTap, Dot11Beacon
from subprocess import call
from time import time, sleep
from numpy import argmax, array
from shared_types import RssiTime
import logging

def freq_to_ch(
        freq_mhz: int
        ) -> Optional[int]: 
    '''
    Convert a frequency to a wifi channel.
    '''
    if freq_mhz < 2401 or freq_mhz > 2495: 
        return None
    if freq_mhz > 2477:
        return 14

    offset = freq_mhz - 2412
    ch = int(float(offset)/5.0) + 1

    if ch < 1: 
        ch = 1
    return ch

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


class WifiRssiMonitor: 
    '''
    Class to monitor WiFi packets for Rssi.
    '''
    __wifi: WifiInterface
    _rssi_list: list[float]
    _time_list: list[float]
    _channel_max_rssi_dict: dict[int, float]
    __monitor_thread: Thread
    _seconds_of_data: int
    _ssid: str

    def __init__(
            self, 
            wifi: WifiInterface, 
            ssid: str,
            kill_processes: bool,
            ) -> None: 
        '''
        Contructor. Starts monitor on given channel.
        '''
        self._ssid = ssid
        self.__wifi = wifi
        self.__wifi.start_monitor_mode(kill_processes)

        # start on channel 1
        self.__wifi.set_channel(1)
        self._channel_max_rssi_dict = {}
        self._rssi_list = []
        self._time_list = []
        self._seconds_of_data = 2

        logging.info("Starting packet capture.")
        self.__monitor_thread = Thread(target=self.__monitor, daemon=True)
        self.__monitor_thread.start()

        logging.info("Starting channel hopping.")
        self.__channel_hop_thread = Thread(target=self.__channel_hop, daemon=True)
        self.__channel_hop_thread.start()

    def __next_channel(
            self,
            current_channel: int
            ) -> int: 
        '''
        Calculate the next channel.
        '''
        if current_channel == 13: 
            return 1 

        return current_channel + 1

    def __channel_hop(
            self
            ) -> None: 
        '''
        Run as background thread to hop channels.
        '''

        current_ch = self.__scan_all_channels_until_beacon_found(0.2)
        scan_ch = self.__next_channel(current_ch)
        while True: 
            self._channel_max_rssi_dict.clear()

            scan_ch = self.__next_channel(scan_ch)
            if scan_ch == current_ch:
                scan_ch = self.__next_channel(current_ch)

            # primary scan 
            logging.debug(f"Primary scan for next 0.8 seconds on channel {current_ch}")
            self.__wifi.set_channel(current_ch)
            sleep(0.8)
            current_ch_rssi = self._channel_max_rssi_dict.get(current_ch)

            # secondary scan
            logging.debug(f"Secondary scan for next 0.2 seconds on channel {scan_ch}")
            self.__wifi.set_channel(scan_ch)
            sleep(0.2) 
            scan_ch_rssi = self._channel_max_rssi_dict.get(scan_ch)

            # eval channel switch
            if scan_ch_rssi: 
                if current_ch_rssi: 
                    if scan_ch_rssi > current_ch_rssi:
                        logging.info(f"Switching to channel {scan_ch} because RSSI is stronger than on channel {current_ch}")
                        current_ch = scan_ch
                else: 
                    logging.info(f"Switching to channel {scan_ch} because there was no rssi data on channel {current_ch}")
                    current_ch = scan_ch
        
    def __find_max_rssi_ch(
            self,
            ) -> Optional[int]: 
        '''
        Finds the channel with the max rssi in dict currently.
        '''
        max_rssi = None 
        current_ch = None
        for ch, rssi in self._channel_max_rssi_dict.items():
            if max_rssi: 
                if rssi > max_rssi: 
                    max_rssi = rssi 
                    current_ch = ch
            else: 
                max_rssi = rssi
                current_ch = ch

        return current_ch

    def __scan_all_channels_until_beacon_found(
            self, 
            sleep_time: float,
            ) -> int: 
        '''
        Performs full scans until a signal is found.
        '''
        current_ch = None
        while not current_ch:
            for ch in range(1, 14):
                logging.debug(f"Scanning channel {ch} for beacon")
                self.__wifi.set_channel(ch)
                sleep(sleep_time)

            current_ch = self.__find_max_rssi_ch()

        logging.info(f"Found strongest beacon on channel {current_ch}")
        return current_ch

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
            ssid = pkt.info.decode()
            if ssid != self._ssid:
                return
            radiotap = pkt.getlayer(RadioTap)
            dot11beacon = pkt.getlayer(Dot11Beacon)
            if radiotap and dot11beacon:
                rssi = float(radiotap.dBm_AntSignal)
                time = float(pkt.time)
                self._rssi_list.append(rssi) 
                self._time_list.append(time)

                ch = dot11beacon.network_stats().get("channel")
                if ch:
                    current_rssi = self._channel_max_rssi_dict.get(ch)
                    if current_rssi:
                        if current_rssi > rssi:
                            self._channel_max_rssi_dict[ch] = rssi
                    else: 
                        self._channel_max_rssi_dict[ch] = rssi
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
            self, 
            start_network_manager: bool
            ) -> None:
        '''
        Stop monitoring.
        '''
        logging.info("Stopping packet capture.") 
        self.__wifi.stop_monitor_mode(start_network_manager)
