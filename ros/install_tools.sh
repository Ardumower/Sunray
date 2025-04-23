#!/usr/bin/env bash

cd /root
echo "==> current folder: $PWD"
ls 

if [ -f Sunray/ros/fix_python_pcl.patch ]; then
   echo "Patch file exists."
else
   echo "Error: this must be run inside docker container"
   exit
fi

#apt update 
#apt-get -y install mplayer psmisc pulseaudio libnotify-bin libsox-fmt-all sox alsa-utils
#exit

apt update && DEBIAN_FRONTEND=noninteractive TZ=Etc/UTC apt-get -y install tzdata \
    software-properties-common build-essential sudo nano wget curl git subversion cmake python-pip python3-pip pybind11-dev gfortran \
    pcl-tools network-manager bluez bluez-tools libbluetooth-dev python-can python3-can can-utils iproute2 psmisc libsox-fmt-all sox \
    mplayer alsa-utils pulseaudio libnotify-bin net-tools dbus libv4l-dev guvcview x11-apps


# Livox SDK2
git clone https://github.com/Livox-SDK/Livox-SDK2 
cd ./Livox-SDK2/   && \
 mkdir build   &&   \
 cd build  &&  \
 cmake .. && make -j1  &&  \
 sudo make install


pip install scipy python-can==2.0.0 cython


# python-pcl and patch
cd /root
git clone https://github.com/strawlab/python-pcl.git
cp Sunray/ros/fix_python_pcl.patch python-pcl/
cd python-pcl &&  git apply --ignore-space-change --ignore-whitespace fix_python_pcl.patch \
    && python setup.py install

apt install -y ros-melodic-move-base ros-melodic-move-base-msgs \
    ros-melodic-joy ros-melodic-teleop-twist-joy ros-melodic-teb-local-planner ros-melodic-global-planner ros-melodic-gmapping \
    ros-tf2-msgs ros-melodic-usb-cam ros-melodic-apriltag-ros \
    ros-melodic-tf2-sensor-msgs ros-melodic-map-server ros-melodic-pointcloud-to-laserscan ros-melodic-dwa-local-planner \
    ros-melodic-octomap ros-melodic-octomap-ros ros-melodic-hector-trajectory-server ros-melodic-tf-conversions ros-melodic-rviz

cd /root
cp -f Sunray/ros/scripts/reboot.sh /sbin/reboot
cp -f Sunray/ros/scripts/shutdown.sh /sbin/shutdown


