#!/bin/bash
cd ~/ardupilot/ArduCopter/
sim_vehicle.py --mavproxy-args="--streamrate=30" --console -v ArduCopter -f gazebo-iris


