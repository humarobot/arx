#!/bin/bash
sudo ip link set up can0 type can bitrate 1000000
# ip link set up can0
# ip link set can1 type can bitrate 1000000
# ip link set up can1
sudo chmod +777 /dev/ttyUSB0
echo "DONE"
