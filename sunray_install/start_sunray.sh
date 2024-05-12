#!/bin/bash

echo "EUID=$EUID"
echo "PWD=$PWD"

CMD=""


# parse (optional) command line options
if [ $# -ne 0 ]; then
  if [ $1 = "--help" ]; then   
    echo "testspeed     test tire speed controller"
    exit
  fi
  if [ $1 = "--testspeed" ]; then
    CMD="testspeed"        
  fi
fi 


if [ "$EUID" -ne 0 ]
  then echo "Please run as root (sudo)"
  exit
fi

if [ ! -d "/home/pi/sunray_install" ]; then
  echo "run install.sh first!"
  exit
fi

if [[ `pidof sunray` != "" ]]; then
  echo "Sunray linux app already running! Exiting..."
  #exit
fi

# start sunray


#DIR=`echo $PWD`
#echo "current directory:" $DIR

#/home/pi/sunray_install/start_ble.sh

# -----------------------------------------
# disable USB autosuspend for all USB devices and for new devices
echo "--------usb device tree--------"
for d in /sys/bus/usb/devices/*/ ; do
  if [ -f "$d"power/control ]; then  
    echo "$d   " $(udevadm info -a --path $d | grep ATTR{product}) "   power=" $(cat "$d"power/control)   
    # switching off autosuspend
    echo on > "$d"power/control
  fi
done

echo "--------usb settings---------"
# show usb autosuspend status
echo "USB autosuspend (2=on, -1=off): " $(cat /sys/module/usbcore/parameters/autosuspend)
echo "switching off USB autosuspend..."
echo '-1' > /sys/module/usbcore/parameters/autosuspend 

# -----------------------------------------
# stop wifi power saving
until iw wlan0 set power_save off  > /dev/null 2>&1 
do 
  sleep 2
  echo "trying to turn off WiFi power saving..."  
done

# avoid wifi lags (https://github.com/tomaspinho/rtl8821ce/issues/143)
# echo "----WiFi config----"
echo "40 255 255 400 3" > /proc/net/rtl8821cu/wlan0/scan_param

echo 0 | sudo tee /sys/module/8821cu/parameters/rtw_power_mgnt
echo 0 | sudo tee /sys/module/8821cu/parameters/rtw_ips_mode
echo 11 | sudo tee /sys/module/8821cu/parameters/rtw_wireless_mode

#MAC=$(ip link show eth0 | grep link/ether | awk '{print $2}')

# -------------------------------------------
# throttle down max cpu freq
echo "throttle down max cpu freq..."
echo "300000" > /sys/devices/system/cpu/cpufreq/policy0/scaling_max_freq

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
btmgmt -i hci0 name "bananapi"
btmgmt -i hci0 advertising on
btmgmt -i hci0 power on
## btmgmt -i hci0 advinfo
# -----------------------------------------

# wait for children processes (start_ble) to finish
wait

echo "----checking i2c connection----"
echo "Waiting for MPU WHOAMI ID to be present..."
# toggle SCL (pin 11) to unstuck i2c bus
# https://forums.raspberrypi.com/viewtopic.php?t=241491
# https://wiki.banana-pi.org/Banana_Pi_BPI-M4#GPIO_PIN_define
# CON1-P12 	SCL_RECOVER 	GPIO-3 
#for _ in `seq 1 9`; do   
#  sudo gpioset 0 3=0  # make output GND (external pull-up LOW)
#  sleep 0.01
#  sudo gpioget 0 3  # make input (external pull-up HIGH) 
#  sleep 0.01
#done;

# power on IMU via port-expander
# i2cset -y 1 0x21 0x07 0xBF   # configuration port 1: enable IO1.6 as output
# i2cset -y 1 0x21 0x03 0x40   # output port 1: set IO1.6 high-level 

# choosing i2c slave0 via multiplexer (for Alfred-dev-PCB without buzzer)
# i2cset -y 1 0x70 0x01  

# choosing i2c slave4 via multiplexer (for Alfred-dev-PCB with buzzer)
# i2cset -y 1 0x70 0x10  

#for _ in `seq 1 10`; do 
#  # read mpu whoami id (0x68)
#  RES=$(i2cget -y 1 0x69 0x75 2> /dev/null | tr -dc '0-9')
#  #RES="00"
#  if [[ $RES != "" ]]; then
#    if [ $RES != "00" ]; then
#      echo "WHOAMI ID=$RES"  
#      break
#    fi
#  fi
#  echo -n .  
#  sleep 1.0     
# done; 


echo "----waiting for TCP connections to be closed from previous sessions----"
echo "Waiting TCP port 80 to be closed..."
for _ in `seq 1 30`; do 
  RES=$(netstat -ant | grep -w 80)
  #RES=$(lsofs -i:80)
  #RES=$(fuser 80/tcp) 
  if [ -z "$RES" ]; then
    break
  fi
  echo $RES
  # echo -n .  
  sleep 2.0     
done; 


echo "----starting sunray----"
echo "CMD=$CMD"
    
cd /boot/sunray


if [ "$CMD" = "testspeed" ]; then
  ( sleep 25;\ 
    for i in {1..5}; do \
      echo -ne "AT+M,1.0,0\r\n"; \
      sleep 0.5; \
      echo -ne "AT+M,0.0,0\r\n"; \
      sleep 2; \
      echo -ne "AT+M,-1.0,0\r\n"; \
      sleep 0.5; \
      echo -ne "AT+M,0.0,0\r\n"; \
      sleep 2; \
    done ) | /home/pi/sunray_install/sunray   
else
  /usr/bin/stdbuf -oL -eL /home/pi/sunray_install/sunray
fi


# debug mode
# exec gdbserver :1234 /home/pi/sunray_install/sunray "$@"
# debug executable with:  gdb /home/pi/sunray_install/sunray

