#!/bin/bash


# -----------------------------------------
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
#btmgmt -i hci0 name "bananapi $MAC"
btmgmt -i hci0 name "owlRobot"
btmgmt -i hci0 advertising on
btmgmt -i hci0 power on
## btmgmt -i hci0 advinfo
# -----------------------------------------

