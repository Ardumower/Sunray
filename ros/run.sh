#!/usr/bin/env bash


# build Sunray ROS node
rm -Rf build
rm -Rf devel
catkin_make


echo "----bluetooth devices----"
hcitool dev
# configure bluetooth BLE module
echo "----BLE config----"
echo 12 > /sys/kernel/debug/bluetooth/hci0/conn_min_interval  # 24   6
echo 20 > /sys/kernel/debug/bluetooth/hci0/conn_max_interval  # 40   6
echo 1 > /sys/kernel/debug/bluetooth/hci0/conn_latency       # 0    1
btmgmt -i hci0 power off
btmgmt -i hci0 le on
btmgmt -i hci0 bredr off
btmgmt -i hci0 connectable on
btmgmt -i hci0 name "ROS robot"
btmgmt -i hci0 advertising on
btmgmt -i hci0 power on


# setup CAN bus
ip link set can0 up type can bitrate 1000000    


echo "---------all processes using the CAN bus------------"
sudo lsof | grep -i can_raw
echo "----------------------------------------------------"


# run Sunray ROS node 
sudo setcap 'cap_net_bind_service=+ep' devel/lib/sunray_node/sunray_node
. devel/setup.bash
roslaunch sunray_node test.launch 



