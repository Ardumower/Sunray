#!/bin/bash

# diagnostics 
#  +filters out and shows errors from kernel log 
#  +shows and configures usb devices
#  +generates usb-camera load (USB load test)
#  +monitors cpu temperature

echo "EUID=$EUID"
echo "PWD=$PWD"

CMD=""


if [ "$EUID" -ne 0 ]
  then echo "Please run as root (sudo)"
  exit
fi

#if [ ! -d "/etc/motion" ]; then
#  echo installing motion...
#  apt install motion
#fi


CMD=""


# parse (optional) command line options
if [ $# -ne 0 ]; then
  if [ $1 = "--help" ]; then   
    echo "--usb     generate usb load"
    echo "--net     generate net load"
    echo "--cap     capture usb packets"
    exit
  fi
  if [ $1 = "--usb" ]; then
    CMD="usb"        
  elif [ $1 = "--net" ]; then
    CMD="net"
  elif [ $1 = "--cap" ]; then
    CMD="cap"  
  fi
fi 


# turn off wifi power safe
echo "switching off WiFi power saving..."
iw wlan0 set power_save off  

# show connected usb cams
echo "--------usb cams---------"
v4l2-ctl --list-devices
v4l2-ctl --list-formats-ext

echo "--------disk devices---------"
df

echo "--------active TCP connections"
netstat -A inet -p

# show hardware
echo "--------hardware--------"
lshw

echo "--------usb devices---------"
cat /sys/kernel/debug/usb/devices | grep -E "^([TSPD]:.*|)$"


# https://logfile.ch/linux/2017/06/15/disable-usb-autosuspend-linux/ 
echo "--------usb device tree--------"
for d in /sys/bus/usb/devices/*/ ; do
  if [ -f "$d"power/control ]; then  
    echo "$d   " $(udevadm info -a --path $d | grep ATTR{product}) "   power=" $(cat "$d"power/control)   
    # switching off autosuspend
    echo on > "$d"power/control
  fi
done

# show usb devices
echo "--------usb settings---------"
# show usb autosuspend status
echo "USB autosuspend (2=on, -1=off): " $(cat /sys/module/usbcore/parameters/autosuspend)

echo "switching off USB autosuspend..."
echo '-1' > /sys/module/usbcore/parameters/autosuspend 


# restart usb host
#cd /sys/bus/pci/drivers/xhci_hcd
#echo -n "0000:00:14.0" > unbind
#echo -n "0000:00:14.0" > bind


# show kernel errors  
echo "--------kernel errors----------"
#cat /var/log/messages | grep "xhci-hcd.4.auto"
cat /var/log/messages | grep "dying"


# cpu freq
echo "--------cpu freq--------"
echo "cpu min freq: "
cat /sys/devices/system/cpu/cpufreq/policy0/scaling_min_freq
echo "cpu max freq: "
cat /sys/devices/system/cpu/cpufreq/policy0/scaling_max_freq
echo "cpu avail freq: "
cat /sys/devices/system/cpu/cpufreq/policy0/scaling_available_frequencies

# throttle down max cpu freq
echo "1400000" > /sys/devices/system/cpu/cpufreq/policy0/scaling_max_freq

echo "--------monitoring cpu temperature---------"

if [ "$CMD" = "usb" ]; then
  echo "generating usb load..."
  streamer -c /dev/video0 -f rgb24 -s 640x480 -r 5 -t 60:00:00 -o /dev/null     
elif [ "$CMD" = "net" ]; then
  echo "generating net load..."
  iperf -c 192.168.2.27 -P 1 -t 3000 -w 1K   
elif [ "$CMD" = "cap" ]; then
  echo "capturing usb packets..."      
  rm /tmp/*.pcap
  tshark -i usbmon0 -b duration:3600 -w /tmp/usb.pcap     
else
  for i in {1..10000}
  do
    paste <(cat /sys/class/thermal/thermal_zone*/type) <(cat /sys/class/thermal/thermal_zone*/temp) | column -s $'\t' -t | sed 's/\(.\)..$/.\1°C/'
    echo CPU freq:
    cat /sys/devices/system/cpu/cpufreq/policy0/scaling_cur_freq
    echo "--------------------------------"
    sleep 2.0
  done
fi



