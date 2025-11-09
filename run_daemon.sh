#!/bin/bash

source /home/aidan/myVenv/bin/activate 
export MAVLINK20=1    # forces pymavlink to use mavlinkv20
python wifi_daemon config.xml
