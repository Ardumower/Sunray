#!/usr/bin/env bash


# build Sunray ROS node
cd build
cmake ..
make

cd build


# setup CAN bus
ip link set can0 up type can bitrate 1000000    


echo "---------all processes using the CAN bus------------"
sudo lsof | grep -i can_raw
echo "----------------------------------------------------"


# run Sunray ROS node 
sudo setcap 'cap_net_bind_service=+ep' devel/lib/sunray_node/sunray_node
. devel/setup.bash
roslaunch sunray_node test.launch 



