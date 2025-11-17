from wifi_rssi import *
from time import sleep
import logging

def main(): 
    card_name = "wlp0s20f3"
    ssid = "beacon"

    logging.basicConfig(handlers=[logging.StreamHandler()], level=logging.DEBUG)

    interface = WifiInterface(card_name)
    monitor = WifiRssiMonitor(interface, ssid, True)

    period = 1
    try:
        while True: 
            sleep(period)

            rssi = monitor.get_max_rssi(period)
            if rssi:
                print(str(rssi))

    except Exception as e: 
        print("Exiting")
        print(e)
    finally:
        monitor.stop()

def print_rssi(rssi: float) -> None: 
    print(rssi)

if __name__ == "__main__": 
    main()
