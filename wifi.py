
from threading import Thread
from pyric import pyw 
from pyshark import LiveCapture
from typing import Callable
from subprocess import call


# callback for received wifi signal
_rssi_cb: Callable[[float], None]


def _handle_packet(pkt) -> None: 
    '''
    Handle a packet 
    '''
    global _rssi_cb
    try:
        _rssi_cb(pkt.radiotap.dbm_antsignal)
    except AttributeError:
        pass


class WifiInterface:
    '''
    Class to control
    '''
    _card_name: str
    _card: pyw.Card

    def __init__(self, card_name: str): 
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

        self._card = pyw.getcard(self._card_name)

    def start_monitor(self, cb: Callable[[float], None]) -> None : 
        '''
        Enter monitor mode, calling a callback 
        every time theres a new RSSI value.

        args: 
            cb: callback
        '''
        global _rssi_cb
        _rssi_cb = cb
        print("Starting monitor mode.")
        if not self._card_name.endswith("mon"):
            # this is the best way i've found to enter monitor mode
            call(["airmon-ng", "check", "kill"]) 
            call(["airmon-ng", "start", self._card_name])
            self._card_name += "mon"
            self._card = pyw.getcard(self._card_name)

        self._capture = LiveCapture(interface=self._card_name)
        print("Starting packet capture.")
        self._capture_thread = Thread(target=self.__monitor, daemon=True)
        self._capture_thread.start()

    def __monitor(self): 
        '''
        Run as background monitor thread.
        '''
        global _handle_packet
        try: 
            self._capture.apply_on_packets(_handle_packet) 
        except: 
            pass

    def stop_monitor(self) -> None:  
        '''
        Exit monitor mode.
        '''
        print("Stoping packet capture.") 
        self._capture.close()
        print("Exiting monitor mode.")
        call(["airmon-ng", "stop", self._card_name])
        call(["systemctl", "start", "NetworkManager"])

    def set_channel(self, channel: int): 
        pyw.chset(self._card, channel)
