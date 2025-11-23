'''
Writes configuration to a file.
'''
import sys
from wifi_daemon import DaemonParams

if len(sys.argv) != 2: 
    print("Please provide the output file as the only argument.")
    sys.exit()

savefile = sys.argv[1]

params = DaemonParams(None)

params.mavlink_connection_string = "/dev/ttyS0"
params.drone_system_id = 1
params.flight_controller_component_id = 0
params.raspberry_pi_component_id = 200 
params.base_station_system_id = 255

params.wifi_card_name = "wlan0" 

params.beacon_period = 1.0
params.beacon_ssid = "beacon"

params.log_directory = "/home/kali/logs/"
params.serial_baud = 115200

params.save(savefile)


# test by reloding from file
params = DaemonParams(savefile)
print("Params after reloading from saved file:")
print(params)
