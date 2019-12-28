#!/bin/bash
cd /home/alfarihfz/Projects/SK-FIX/web_app
tmux new -s web-app -d 'python3 start.py'
#tmux new -s mavproxy -d 'mavproxy.py --master /dev/ttyUSB0 --baudrate 57600 --out udp://127.0.0.1:14550 --quadcopter'
#tmux new -s mavros -d 'roslaunch mavros apm.launch fcu_url:=udp://127.0.0.1:14550@'
