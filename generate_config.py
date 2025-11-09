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

params.mavlink_connection_string = "tcpin:xxx.xxx.xx.xx" 
params.drone_system_id = 1
params.drone_component_id = 0
params.raspberry_pi_component_id = 200 
params.base_station_system_id = 255

params.wifi_card_name = "wlp0s20f3" 
params.wifi_channel = 11

params.beacon_period = 1.0

params.log_directory = "/home/aidan/Avy-Dawgs/Signal-Map/logs/"

params.save(savefile)

# test by reloding from file
params = DaemonParams(savefile)
print("Params after reloading from saved file:")
print(params)
