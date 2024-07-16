# Start from a base image of Ubuntu 18.04 (bionic)
FROM ubuntu:bionic 

RUN apt update && DEBIAN_FRONTEND=noninteractive TZ=Etc/UTC apt-get -y install tzdata \
    software-properties-common build-essential sudo nano wget curl git subversion

# Miniconda
#RUN wget \
#    https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-aarch64.sh \
#    && mkdir /root/.conda \
#    && bash Miniconda3-latest-Linux-aarch64.sh -b \
#    && rm -f Miniconda3-latest-Linux-aarch64.sh 
#RUN conda --version
#RUN source /root/miniconda3/bin/activate && conda init --all && conda create --name py311 python=3.11 && conda create --name py27 -c conda-forge python=2.7

## TODO: adjust PYTHONPATH

# Livox SDK2
RUN git clone https://github.com/Livox-SDK/Livox-SDK2 
RUN cd ./Livox-SDK2/   && \
 mkdir build   &&   \
 cd build  &&  \
 cmake .. && make -j1  &&  \
 sudo make install


RUN pip install scipy python-can==2.0.0


# python-pcl and patch
RUN git clone https://github.com/strawlab/python-pcl.git
COPY fix_python_pcl.patch python-pcl/
RUN cd python-pcl &&  git apply --ignore-space-change --ignore-whitespace fix_python_pcl.patch \
    && python setup.py install



# ROS Melodic
RUN add-apt-repository universe
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
   
# if you haven't already installed curl
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
RUN apt update
#RUN apt install -y ros-melodic-ros-base
RUN apt install -y ros-melodic-desktop-full

# Source the ROS environment setup script
ENV ROS_DISTRO melodic
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc

RUN apt install -y python-pip python3-pip pybind11-dev gfortran ros-melodic-move-base ros-melodic-move-base-msgs \
    ros-melodic-joy ros-melodic-teleop-twist-joy ros-melodic-teb-local-planner ros-melodic-global-planner ros-melodic-gmapping \
    pcl-tools network-manager bluez bluez-tools libbluetooth-dev python-can python3-can can-utils iproute2 ros-tf2-msgs \
    ros-melodic-tf2-sensor-msgs ros-melodic-map-server ros-melodic-pointcloud-to-laserscan ros-melodic-dwa-local-planner \
    ros-melodic-octomap ros-melodic-octomap-ros ros-melodic-hector-trajectory-server




