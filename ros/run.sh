#!/usr/bin/env bash


IMAGE_NAME="ros:melodic-perception-bionic"
CONTAINER_NAME="ros1"
HOST_MAP_PATH=`realpath $PWD/..`
CONFIG_FILE="/root/Sunray/alfred/config_owlmower.h"
#CONFIG_FILE="/root/Sunray/alfred/config_fuxtec_ros.h"
USER_UID=$(id -u)


function docker_install {
  # install docker
  # Add Docker's official GPG key:
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
  if [ "$EUID" -eq 0 ]
    then echo "Please run as non-root (not sudo)"
    exit
  fi
  docker pull "$IMAGE_NAME"
}

function docker_build_container {
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
  echo "HOST_MAP_PATH: $HOST_MAP_PATH"
  echo "IMAGE_NAME: $IMAGE_NAME"
  echo "====> enter 'exit' to exit docker container" 
  docker run --name=$CONTAINER_NAME -t -it --net=host --privileged -v /dev:/dev \
    -v /run/user/0/bus:/run/user/0/bus \
    -e DBUS_SESSION_BUS_ADDRESS="unix:path=/run/user/0/bus" \
    -v /var/run/pulse/native:/root/pulse/native \
    -v /var/run/pulse/.config/pulse/cookie:/root/.config/pulse/cookie \
    -e DISPLAY=$DISPLAY --volume="$HOME/.Xauthority:/root/.Xauthority:rw" -v $HOST_MAP_PATH:/root/Sunray  $IMAGE_NAME   
}

function docker_show_containers {
  if [ "$EUID" -ne 0 ]
    then echo "Please run as root (sudo)"
    exit
  fi
  docker ps -all
}

function docker_terminal {
  if [ "$EUID" -ne 0 ]
    then echo "Please run as root (sudo)"
    exit
  fi
  # -------------continue container...---------------------
  # allow docker to access host Xserver 
  #xhost +local:*
  docker start $CONTAINER_NAME && docker attach $CONTAINER_NAME 
}

function docker_prepare_tools {
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
  if [ "$EUID" -ne 0 ]
    then echo "Please run as root (sudo)"
    exit
  fi  
  # build Sunray ROS node
  docker start $CONTAINER_NAME && docker exec -t -it $CONTAINER_NAME \
    bash -c ". /ros_entrypoint.sh ; cd /root/Sunray/ros/ ; rm -Rf build ; rm -Rf devel ; catkin_make -DCONFIG_FILE=$CONFIG_FILE -DROS_EDITION=ROS1"
}

function ros_run {
  #pulseaudio -k  
  sudo killall pulseaudio
  sleep 1
  pulseaudio -D --system --disallow-exit --disallow-module-loading --verbose
  mplayer /home/pi/Sunray/tts/de/temperature_low_docking.mp3
  echo "--------"  
  #exit

  docker start $CONTAINER_NAME && docker exec -t -it $CONTAINER_NAME \
    bash -c 'killall pulseaudio ; sleep 1 ; export PULSE_SERVER=unix:/root/pulse/native ; export PULSE_COOKIE=/root/.config/pulse/cookie ; \
            export DISPLAY=:0 ; pulseaudio -D --system --disallow-exit --disallow-module-loading --verbose ; \
            mplayer /root/Sunray/tts/de/temperature_low_docking.mp3' 
  exit

  if [ "$EUID" -ne 0 ]
    then echo "Please run as root (sudo)"
    exit
  fi  
  echo "----bluetooth devices----"
  systemctl enable bluetooth.service
  hcitool dev
  # configure bluetooth BLE module
  echo "----BLE config----"
  echo 12 > /sys/kernel/debug/bluetooth/hci0/conn_min_interval  # 24   6
  echo 20 > /sys/kernel/debug/bluetooth/hci0/conn_max_interval  # 40   6
  echo 1 > /sys/kernel/debug/bluetooth/hci0/conn_latency       # 0    1
  echo 153 > /sys/kernel/debug/bluetooth/hci0/adv_min_interval  # 0.625 ms units
  echo 153 > /sys/kernel/debug/bluetooth/hci0/adv_max_interval  # 0.625 ms units
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

  # setup audio interface
  # https://gavv.net/articles/pulseaudio-under-the-hood/
  # https://www.freedesktop.org/wiki/Software/PulseAudio/Documentation/User/SystemWide/
  # https://unix.stackexchange.com/questions/204522/how-does-pulseaudio-start
  if ! command -v play &> /dev/null
  then 
    echo "installing audio player..."
    apt install -y libsox-fmt-mp3 sox mplayer alsa-utils pulseaudio
  fi
  # show audio devices
  #aplay -l
  cat /proc/asound/cards
  # restart pulseaudio daemon as root
  killall pulseaudio
  pulseaudio -D --system --disallow-exit --disallow-module-loading
  # set default volume 
  amixer -D pulse sset Master 100%

  echo "----waiting for TCP connections to be closed from previous sessions----"
  echo "Waiting TCP port 80 to be closed..."
  for _ in `seq 1 20`; do 
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

  # run Sunray ROS node 
  echo "starting sunray ROS node..."
  # allow non-root to start http server 
  #sudo setcap 'cap_net_bind_service=+ep' devel/lib/sunray_node/sunray_node

  # source ROS setup  
  docker start $CONTAINER_NAME && docker exec -t -it $CONTAINER_NAME \
    bash -c 'export ROS_HOME=/root/Sunray/alfred ; . /ros_entrypoint.sh ; cd /root/Sunray/ros ; . devel/setup.bash ;  \ 
            setcap 'cap_net_bind_service=+ep' devel/lib/sunray_node/sunray_node ; cd /root/Sunray/alfred ; pwd ;  \ 
            roslaunch sunray_node test.launch' 
  # rosnode kill -a ; sleep 3
}



PS3='Please enter your choice: '
options=(
    "Docker install"
    "Docker pull ROS image"
    "Docker build container"
    "Docker show containers"
    "Docker run terminal"
    "Docker prepare ROS tools"
    "ROS compile"    
    "ROS run"    
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
        "ROS compile")
            ros_compile
            break
            ;;
        "ROS run")
            ros_run
            break
            ;;
        "Quit")
            break
            ;;
        *) echo "invalid option $REPLY";;
    esac
done



