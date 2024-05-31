#!/bin/bash

source /opt/ros/humble/setup.bash
source /home/dev/test-bonehead/install/setup.bash

/opt/ros/humble/bin/ros2 launch bonehead bonehead_hw.launch.py
