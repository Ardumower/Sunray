#!/usr/bin/env bash

# -pull ROS docker image (ros:melodic-perception-bionic)
# -build ROS docker container
# -compile Sunray ROS node
# -start/stop Sunray ROS node service
# -start/stop Sunray ROS LiDAR mapping
# -start Sunray ROS LiDAR localization


# test camera:  guvcview --device=/dev/video2 --format=mjpeg


IMAGE_NAME="ros:melodic-perception-bionic"
CONTAINER_NAME="ros1"

HOST_SUNRAY_PATH=`realpath $PWD/..`
HOST_PCD_PATH=`realpath $PWD/../../PCD`

#CONFIG_FILE="/root/Sunray/alfred/config_owlmower.h"
#CONFIG_FILE="/root/Sunray/alfred/config_fuxtec_ros.h"
USER_UID=$(id -u)
BAG_FILE=/root/PCD/playback.bag

MAP_RES="0.25"
#MAP_RES="0.01"

if [ -z "$SUNRAY_ROS_RVIZ" ]; then
  SUNRAY_ROS_RVIZ=true
fi

if [ -z "$USE_BAG_FILE" ]; then
  USE_BAG_FILE=false
fi


function prepare_for_ros {
  echo "prepare_for_ros"
  WCON=$(nmcli c | grep wifi | head -1 | tail -c 12 | xargs )
  echo "WIFI CON: $WCON"
  WIP=`ifconfig $WCON | grep 'inet ' | awk -F'[: ]+' '{ print $3 }'`
  echo "WIFI IP: $WIP"
  if [[ $WIP != "" ]]; then
    export ROS_IP=`ifconfig $WCON | grep 'inet ' | awk -F'[: ]+' '{ print $3 }'`
  else
    unset ROS_IP
  fi
  # allow docker to access host Xserver 
  xhost +local:* 

  # show audio devices
  if ! command -v play &> /dev/null
  then 
    echo "installing audio player..."
    apt install -y libsox-fmt-mp3 sox mplayer alsa-utils pulseaudio
  fi  
  #aplay -l
  cat /proc/asound/cards
  # restart pulseaudio daemon as root
  #pulseaudio -k
  #killall pulseaudio
  #sleep 1
  #pulseaudio -D --system --disallow-exit --disallow-module-loading
  #sudo chmod 666 /var/run/pulse/native
  #export PULSE_SERVER=unix:/var/run/pulse/native
  # set default volume 
  #adduser root dialout audio pulse-access pulse
  #amixer -D pulse sset Master 100%
  #   echo "====> we will test host audio now... (you should hear a voice)" 
  #   mplayer /home/pi/Sunray/tts/de/system_starting.mp3
  #exit
}


function docker_install {
  # install docker
  # Add Docker's official GPG key:
  echo "docker_install"
  sudo apt-get update
  sudo apt-get install ca-certificates curl
  sudo install -m 0755 -d /etc/apt/keyrings
  sudo curl -fsSL https://download.docker.com/linux/debian/gpg -o /etc/apt/keyrings/docker.asc
  sudo chmod a+r /etc/apt/keyrings/docker.asc

  # Add the repository to Apt sources:
  echo \
    "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/debian \
    $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
    sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
  sudo apt-get update

  sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
}

function docker_pull_image {
  # -----------pull image...------------------------    
  echo "docker_pull_image"
  if [ "$EUID" -ne 0 ]
    then echo "Please run as root (sudo)"
    exit
  fi
  docker pull "$IMAGE_NAME"
}

function docker_build_container {
  echo "docker_build_container"
  # -----------create container...------------------------  
  #if [ "$EUID" -eq 0 ]
  #  then echo "Please run as non-root (not sudo)"
  #  exit
  #fi
  if [ "$EUID" -ne 0 ]
    then echo "Please run as root (sudo)"
    exit
  fi  
  echo "HOME: $HOME"
  echo "XDG_RUNTIME_DIR: $XDG_RUNTIME_DIR"
  echo "HOST_SUNRAY_PATH: $HOST_SUNRAY_PATH"
  echo "HOST_PCD_PATH: $HOST_PCD_PATH"  
  echo "IMAGE_NAME: $IMAGE_NAME"
  echo "DISPLAY: $DISPLAY"
  #exit
  prepare_for_ros  
  echo "====> enter 'exit' to exit docker container"   
  docker run --name=$CONTAINER_NAME -t -it --net=host --privileged -v /dev:/dev \
    --volume /etc/machine-id:/etc/machine-id:ro \
    --volume /var/run/dbus:/var/run/dbus \
    --device /dev/snd \
    -e DISPLAY=$DISPLAY --volume="$HOME/.Xauthority:/root/.Xauthority:rw" -v $HOST_PCD_PATH:/root/PCD -v $HOST_SUNRAY_PATH:/root/Sunray  $IMAGE_NAME
}

function docker_show_containers {
  echo "docker_show_containers"
  if [ "$EUID" -ne 0 ]
    then echo "Please run as root (sudo)"
    exit
  fi
  docker ps -all
}

function docker_image_from_container {
  echo "docker_image_from_container"
  if [ "$EUID" -ne 0 ]
    then echo "Please run as root (sudo)"
    exit
  fi
  docker commit $CONTAINER_NAME ros:melodic-perception-bionic-modified
  docker images -a 
}

function docker_terminal {
  echo "docker_terminal"
  if [ "$EUID" -ne 0 ]
    then echo "Please run as root (sudo)"
    exit
  fi
  # -------------continue container...---------------------
  prepare_for_ros
  CMD=". /ros_entrypoint.sh"
  if [[ $WIP != "" ]]; then
    CMD+="; export ROS_IP=$WIP"
  fi
  CMD+="; export DISPLAY=$DISPLAY"
  CMD+="; export ROS_HOME=/root/Sunray/alfred ; cd /root/Sunray/ros ; . devel/setup.bash ; /bin/bash"
  docker start $CONTAINER_NAME && docker exec -t -it $CONTAINER_NAME \
    bash -c "$CMD"
}

function docker_prepare_tools {
  echo "docker_prepare_tools"
  if [ "$EUID" -ne 0 ]
    then echo "Please run as root (sudo)"
    exit
  fi  
  # -------------continue container...---------------------
  prepare_for_ros    
  docker start $CONTAINER_NAME && docker exec -t -it $CONTAINER_NAME /root/Sunray/ros/install_tools.sh
}

function ros_compile {
  echo "ros_compile"
  if [ "$EUID" -ne 0 ]
    then echo "Please run as root (sudo)"
    exit
  fi 

  PS3='Please choose config: '
  list=$(ls ../alfred/config*.h)
  IFS=$'\n'
  select CONFIG_FILE in $list 
    do test -n "$CONFIG_FILE" && break; 
  exit; 
  done
  echo "selected: $CONFIG_FILE"
  CONFIG_PATHNAME=/root/Sunray/alfred/$CONFIG_FILE

  # build single package:    catkin_make -DCATKIN_WHITELIST_PACKAGES="ground_lidar_processor"
  # build Sunray ROS node
  prepare_for_ros    
  CMD=". /ros_entrypoint.sh ;"
  CMD+="cd /root/Sunray/ros/ ;"
  CMD+="rm -Rf build ; rm -Rf devel ;"
  CMD+="catkin_make -DCONFIG_FILE=$CONFIG_PATHNAME -DROS_EDITION=ROS1 -DCMAKE_BUILD_TYPE=Release"
  docker start $CONTAINER_NAME && docker exec -t -it $CONTAINER_NAME \
    bash -c "$CMD"
}

function ros_recompile {
  echo "ros_recompile"
  if [ "$EUID" -ne 0 ]
    then echo "Please run as root (sudo)"
    exit
  fi 
  # build single package:    catkin_make -DCATKIN_WHITELIST_PACKAGES="ground_lidar_processor"
  # build Sunray ROS node
  prepare_for_ros    
  CMD=". /ros_entrypoint.sh ;"
  CMD+="cd /root/Sunray/ros/ ;"
  CMD+="catkin_make"
  docker start $CONTAINER_NAME && docker exec -t -it $CONTAINER_NAME \
    bash -c "$CMD"
}

function rviz {
  echo "rviz"
  # export ROS_MASTER_URI=http://raspberrypi.local:11311
  #export ROS_IP=testpi5.local
  #rviz -d src/pcl_docking/rviz/pcl_docking.rviz
  #rviz -d src/direct_lidar_odometry/launch/dlo_mid360.rviz
  #rviz -d src/ground_lidar_processor/launch/test.rviz
  # allow docker to access host Xserver 
  prepare_for_ros
  docker start $CONTAINER_NAME && docker exec -t -it $CONTAINER_NAME \
    bash -c ". /ros_entrypoint.sh ; cd /root/Sunray/ros/ ; export DISPLAY=$DISPLAY ; export QT_QPA_PLATFORM=xcb ; rviz -d src/ground_lidar_processor/launch/test.rviz"
}



function ros_test_lidar {
  echo "ros_test_lidar"
  export SUNRAY_ROS_MODE=TEST_LIDAR
  export SUNRAY_ROS_RVIZ=$SUNRAY_ROS_RVIZ
  sudo -E ./start_sunray_ros.sh
}


function ros_test_ground_bumper {
  if [ "$EUID" -ne 0 ]
    then echo "Please run as root (sudo)"
    exit
  fi
  echo "ros_test_ground_bumper"
  export SUNRAY_ROS_MODE=SIMPLE
  export SUNRAY_ROS_RVIZ=$SUNRAY_ROS_RVIZ
  sudo -E ./start_sunray_ros.sh
}



function ros_start_mapping {
  if [ "$EUID" -ne 0 ]
    then echo "Please run as root (sudo)"
    exit
  fi
  echo "ros_start_mapping"
  export SUNRAY_ROS_MODE=MAPPING
  export SUNRAY_ROS_RVIZ=$SUNRAY_ROS_RVIZ
  export USE_BAG_FILE=$USE_BAG_FILE
  export BAG_FILE=$BAG_FILE
  sudo -E ./start_sunray_ros.sh
}


function ros_stop_mapping {
    if [ "$EUID" -ne 0 ]
      then echo "Please run as root (sudo)"
      exit
    fi
    echo "ros_stop_mapping"
    prepare_for_ros

    CMD=""    
    if [[ $WIP != "" ]]; then
      CMD+="export ROS_IP=$WIP"
    fi
    CMD+="; export ROS_HOME=/root/Sunray/alfred"
    CMD+="; . /ros_entrypoint.sh"
    CMD+="; cd /root/Sunray/ros"
    CMD+="; . devel/setup.bash"
    CMD+="; setcap 'cap_net_bind_service=+ep' devel/lib/sunray_node/sunray_node"
    CMD+="; cd /root/Sunray/alfred"
    CMD+="; pwd"
    CMD+="; rosservice call /robot/dlio_map/save_pcd $MAP_RES /root/PCD"
    CMD+="; ls -la /root/PCD"
    docker start $CONTAINER_NAME && docker exec -t -it $CONTAINER_NAME bash -c "$CMD"
    
    docker stop $CONTAINER_NAME

    # rosservice call /robot/dlo_map/save_pcd LEAF_SIZE SAVE_PATH
    #rosservice call /robot/dlio_map/save_pcd 0.25 $PCDDIR
    # mv $PCDDIR/dlio_map.pcd $PCDMAP 
    # rosservice call /robot/dlo_odom/save_traj SAVE_PATH

    # rostopic pub /topic_name std_msgs/String $PCDDIR    
    
    ## rostopic pub syscommand std_msgs/String "savegeotiff"
    # rosrun map_server map_saver -f indoor
    # rosrun map_server map_server my_map.yaml
}


function ros_sunray {
  echo "ros_sunray"
  #export SUNRAY_ROS_MODE=SIMPLE
  #export SUNRAY_ROS_MODE=LOCALIZATION  
  export SUNRAY_ROS_RVIZ=$SUNRAY_ROS_RVIZ
  export USE_BAG_FILE=$USE_BAG_FILE
  export BAG_FILE=$BAG_FILE
  sudo -E ./start_sunray_ros.sh
}

function ros_trigger_relocalization {
  echo "ros_trigger_relocalization"
  prepare_for_ros
  CMD=""
  if [[ $WIP != "" ]]; then
    CMD+="export ROS_IP=$WIP"
  fi
  CMD+="; export ROS_HOME=/root/Sunray/alfred"
  CMD+="; . /ros_entrypoint.sh ; cd /root/Sunray/ros ; . devel/setup.bash"
  CMD+="; setcap 'cap_net_bind_service=+ep' devel/lib/sunray_node/sunray_node"
  CMD+="; cd /root/Sunray/alfred ; pwd ; rosservice call /global_localization ; rostopic echo /mcl_3dl/status"
  docker start $CONTAINER_NAME && docker exec -t -it $CONTAINER_NAME \
    bash -c "$CMD"
}

function toggle_ros_ip {
  if [ -z "$ROS_IP" ]; then
    WCON=$(nmcli c | grep wifi | head -1 | tail -c 12 | xargs )
    echo "WIFI CON: $WCON"
    WIP=`ifconfig $WCON | grep 'inet ' | awk -F'[: ]+' '{ print $3 }'`
    echo "WIFI IP: $WIP"
    if [[ $WIP != "" ]]; then
      echo "setting ROS_IP"
      export ROS_IP=`ifconfig $WCON | grep 'inet ' | awk -F'[: ]+' '{ print $3 }'`
    fi  
  else
    echo "unsetting ROS_IP"
    unset ROS_IP
  fi
}


function toggle_rviz {
  if [[ "$SUNRAY_ROS_RVIZ" == "true" ]]; then
    SUNRAY_ROS_RVIZ=false
  else
    SUNRAY_ROS_RVIZ=true
  fi
}

function toggle_use_bag_file {
  if [[ "$USE_BAG_FILE" == "true" ]]; then
    USE_BAG_FILE=false
  else
    USE_BAG_FILE=true
  fi
}

function start_sunray_ros_service {
  if [ "$EUID" -ne 0 ]
    then echo "Please run as root (sudo)"
    exit
  fi  
  if [[ `pidof sunray` != "" ]]; then
        echo "Sunray linux app already running! Exiting..."
        exit
  fi
  # enable sunray service
  echo "starting sunray ROS service..."
  #ln -s /home/pi/sunray_install/config_files/sunray.service /etc/systemd/system/sunray.service
  cp $PWD/sunray_ros.service /etc/systemd/system/sunray_ros.service
  chmod 644 /etc/systemd/system/sunray_ros.service
  mkdir -p /boot/sunray
  chmod 644 /boot/sunray
  systemctl daemon-reload
  systemctl enable sunray_ros
  systemctl start sunray_ros
  systemctl --no-pager status sunray_ros
  echo "sunray ROS service started!"
}

function stop_sunray_ros_service {
  if [ "$EUID" -ne 0 ]
    then echo "Please run as root (sudo)"
    exit
  fi
  # disable sunray service
  echo "stopping sunray ROS service..."
  docker stop $CONTAINER_NAME
  systemctl stop sunray_ros
  systemctl disable sunray_ros
  echo "sunray ROS service stopped!"
}

function showlog(){
  echo "show log..."
  journalctl -f -u sunray_ros
}

function savelog(){
  echo "saving log.txt..."
  journalctl -u sunray_ros > log.txt
}


function menu {
  echo "SUNRAY_ROS_RVIZ: $SUNRAY_ROS_RVIZ"
  echo "USE_BAG_FILE: $USE_BAG_FILE"
  echo "ROS_IP: $ROS_IP"
  echo "HOME: $HOME"
  echo "XDG_RUNTIME_DIR: $XDG_RUNTIME_DIR"
  echo "HOST_SUNRAY_PATH: $HOST_SUNRAY_PATH"
  echo "HOST_PCD_PATH: $HOST_PCD_PATH"  
  echo "IMAGE_NAME: $IMAGE_NAME"
  echo "DISPLAY: $DISPLAY"
  PS3='Please enter your choice: '
  options=(
      "Docker install"
      "Docker pull ROS image"
      "Docker build container"
      "Docker show containers"
      "Docker create image from container"
      "Docker prepare ROS tools"
      "Docker run terminal"
      "ROS compile" 
      "ROS recompile" 
      "rviz"
      "ROS test LiDAR"
      "ROS test ground bumper"    
      "ROS start LiDAR mapping"
      "ROS stop LiDAR mapping"    
      "ROS run sunray"
      "ROS trigger re-localization"
      "toggle SUNRAY_ROS_RVIZ"
      "toggle USE_BAG_FILE"
      "toggle ROS_IP"
      "Start sunray ROS service"
      "Stop sunray ROS service"
      "Show sunray ROS service log"
      "Save sunray ROS service log"
      "Quit")
  select opt in "${options[@]}"
  do
      case $opt in
          "Docker install")
              docker_install
              break
              ;;
          "Docker pull ROS image")        
              docker_pull_image
              break
              ;;
          "Docker build container")
              docker_build_container
              break
              ;;
          "Docker show containers")
              docker_show_containers
              break
              ;;            
          "Docker prepare ROS tools")
              docker_prepare_tools
              break
              ;;
          "Docker run terminal")
              docker_terminal
              break
              ;;            
          "Docker create image from container")
              docker_image_from_container
              break
              ;;
          "ROS compile")
              ros_compile
              break
              ;;
          "ROS recompile")
              ros_recompile
              break
              ;;
          "rviz")
              rviz
              break
              ;;
          "ROS test LiDAR")
              ros_test_lidar
              break
              ;;
          "ROS test ground bumper")
              ros_test_ground_bumper
              break
              ;;
          "ROS run sunray simple")
              ros_sunray_simple
              break
              ;;
          "ROS start LiDAR mapping")
              ros_start_mapping
              break
              ;;
          "ROS stop LiDAR mapping")
              ros_stop_mapping
              break
              ;;
          "ROS run sunray")
              ros_sunray
              break
              ;;
          "ROS trigger re-localization")
              ros_trigger_relocalization
              break
              ;;
          "toggle SUNRAY_ROS_RVIZ")
              toggle_rviz
              menu
              break
              ;;
          "toggle USE_BAG_FILE")
              toggle_use_bag_file
              menu
              break
              ;;              
          "toggle ROS_IP")
              toggle_ros_ip
              menu
              break
              ;;              
          "Start sunray ROS service")
              start_sunray_ros_service
              break
              ;;
          "Stop sunray ROS service")
              stop_sunray_ros_service
              break
              ;;            
          "Show sunray ROS service log")
              showlog
              break
              ;;     
          "Save sunray ROS service log")
              savelog
              break
              ;;
          "Quit")
              break
              ;;
          *) echo "invalid option $REPLY";;
      esac
  done
}


menu
