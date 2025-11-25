#!/bin/bash

source /home/kali/the-venv/bin/activate 
# export MAVLINK20=1    # forces pymavlink to use mavlinkv20
python /home/kali/Signal-Map/wifi_daemon.py config.xml
