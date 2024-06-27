#!/usr/bin/env bash


# build Sunray ROS node
cmake ..
make


# run Sunray ROS node 
sudo setcap 'cap_net_bind_service=+ep' devel/lib/sunray_node/sunray_node

roslaunch sunray_node test.launch 



