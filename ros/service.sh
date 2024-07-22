#!/usr/bin/env bash

# -pull ROS docker image (ros:melodic-perception-bionic)
# -build ROS docker container
# -compile Sunray ROS node
# -start/stop Sunray ROS node service


IMAGE_NAME="ros:melodic-perception-bionic"
CONTAINER_NAME="ros1"

HOST_SUNRAY_PATH=`realpath $PWD/..`
HOST_PCD_PATH=`realpath $PWD/../../PCD`

#CONFIG_FILE="/root/Sunray/alfred/config_owlmower.h"
#CONFIG_FILE="/root/Sunray/alfred/config_fuxtec_ros.h"
USER_UID=$(id -u)


function export_ros_ip {
  echo "export_ros_ip"
  WCON=$(nmcli c | grep wifi | head -1 | tail -c 12 | xargs )
  echo "WIFI CON: $WCON"
  WIP=`ifconfig $WCON | grep 'inet ' | awk -F'[: ]+' '{ print $3 }'`
  echo "WIFI IP: $WIP"
  export ROS_IP=`ifconfig $WCON | grep 'inet ' | awk -F'[: ]+' '{ print $3 }'`
  # allow docker to access host Xserver 
  xhost +local:* 
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
  if [ "$EUID" -eq 0 ]
    then echo "Please run as non-root (not sudo)"
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
  echo "====> enter 'exit' to exit docker container" 
  #exit
  docker run --name=$CONTAINER_NAME -t -it --net=host --privileged -v /dev:/dev \
    --env PULSE_SERVER=unix:/tmp/pulse_socket \
    --volume /tmp/pulse_socket:/tmp/pulse_socket \
    --volume /etc/machine-id:/etc/machine-id:ro \
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
  export_ros_ip
  docker start $CONTAINER_NAME && docker exec -t -it $CONTAINER_NAME \
    bash -c ". /ros_entrypoint.sh ; export ROS_IP=$WIP ; export ROS_HOME=/root/Sunray/alfred ; cd /root/Sunray/ros ; . devel/setup.bash ; /bin/bash"
}

function docker_prepare_tools {
  echo "docker_prepare_tools"
  if [ "$EUID" -ne 0 ]
    then echo "Please run as root (sudo)"
    exit
  fi  
  # -------------continue container...---------------------
  # allow docker to access host Xserver 
  #xhost +local:*
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

  # build Sunray ROS node
  docker start $CONTAINER_NAME && docker exec -t -it $CONTAINER_NAME \
    bash -c ". /ros_entrypoint.sh ; cd /root/Sunray/ros/ ; rm -Rf build ; rm -Rf devel ; catkin_make -DCONFIG_FILE=$CONFIG_PATHNAME -DROS_EDITION=ROS1"
}

function rviz {
  echo "rviz"
  # export ROS_MASTER_URI=http://raspberrypi.local:11311
  #export ROS_IP=testpi5.local
  #rviz -d src/pcl_docking/rviz/pcl_docking.rviz
  #rviz -d src/direct_lidar_odometry/launch/dlo_mid360.rviz
  #rviz -d src/ground_lidar_processor/launch/test.rviz
  # allow docker to access host Xserver 
  export_ros_ip
  docker start $CONTAINER_NAME && docker exec -t -it $CONTAINER_NAME \
    bash -c ". /ros_entrypoint.sh ; cd /root/Sunray/ros/ ; export QT_QPA_PLATFORM=xcb ; rviz -d src/ground_lidar_processor/launch/test.rviz"
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
  systemctl stop sunray_ros
  systemctl disable sunray_ros
  echo "sunray ROS service stopped!"
}

function ros_test_lidar {
  echo "ros_test_lidar"
  export SUNRAY_ROS_MODE=TEST_LIDAR
  export SUNRAY_ROS_RVIZ=true
  sudo -E ./start_sunray_ros.sh
}

function ros_sunray_simple {
  echo "ros_sunray_simple"
  export SUNRAY_ROS_MODE=SIMPLE
  export SUNRAY_ROS_RVIZ=false
  sudo -E ./start_sunray_ros.sh
}

function ros_start_mapping {
  echo "ros_start_mapping"
  export SUNRAY_ROS_MODE=MAPPING
  export SUNRAY_ROS_RVIZ=true
  sudo -E ./start_sunray_ros.sh
}


function ros_stop_mapping {
    echo "ros_stop_mapping"
    export_ros_ip
    docker start $CONTAINER_NAME && docker exec -t -it $CONTAINER_NAME \
      bash -c "export ROS_IP=$WIP ; export ROS_HOME=/root/Sunray/alfred ; . /ros_entrypoint.sh ; cd /root/Sunray/ros ; . devel/setup.bash ; setcap 'cap_net_bind_service=+ep' devel/lib/sunray_node/sunray_node ; cd /root/Sunray/alfred ; pwd ; rosservice call /robot/dlio_map/save_pcd 0.25 /root/PCD ; ls -la /root/PCD"
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


function ros_sunray_localization {
  echo "ros_sunray_localization"
  export SUNRAY_ROS_MODE=LOCALIZATION
  export SUNRAY_ROS_RVIZ=true
  sudo -E ./start_sunray_ros.sh
}

function ros_trigger_relocalization {
  echo "ros_trigger_relocalization"
  export_ros_ip
  docker start $CONTAINER_NAME && docker exec -t -it $CONTAINER_NAME \
    bash -c "export ROS_IP=$WIP ; export ROS_HOME=/root/Sunray/alfred ; . /ros_entrypoint.sh ; cd /root/Sunray/ros ; . devel/setup.bash ; setcap 'cap_net_bind_service=+ep' devel/lib/sunray_node/sunray_node ; cd /root/Sunray/alfred ; pwd ; rosservice call /global_localization ; rostopic echo /mcl_3dl/status"
}


PS3='Please enter your choice: '
options=(
    "Docker install"
    "Docker pull ROS image"
    "Docker build container"
    "Docker show containers"
    "Docker create image from container"
    "Docker run terminal"
    "Docker prepare ROS tools"
    "ROS compile"    
    "rviz"
    "Start sunray ROS service"
    "Stop sunray ROS service"
    "ROS run test LiDAR"    
    "ROS run sunray simple"
    "ROS start LiDAR mapping"
    "ROS stop LiDAR mapping"    
    "ROS run sunray localization"
    "ROS trigger re-localization"
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
        "Docker run terminal")
            docker_terminal
            break
            ;;            
        "Docker prepare ROS tools")
            docker_prepare_tools
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
        "rviz")
            rviz
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
        "ROS run test LiDAR")
            ros_test_lidar
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
        "ROS run sunray localization")
            ros_sunray_localization
            break
            ;;
        "ROS trigger re-localization")
            ros_trigger_relocalization
            break
            ;;
        "Quit")
            break
            ;;
        *) echo "invalid option $REPLY";;
    esac
done



