#!/bin/bash
set -e
roscore &
roslaunch rosbridge_server rosbridge_websocket.launch &
python2 sparki-ros.py /dev/ttyACM0 &
python2 main.py

