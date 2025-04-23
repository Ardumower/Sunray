# pointcloud_gravity_align

## 1 Introduction
This is a ROS tool package to align imu and lidar frame's z axis with the gravity based on imu data when robot is **STATIC**.

## 2 Prerequisites
### 2.1 Ubuntu and ROS
### 2.2 PCL & Eigen
- PCL >= 1.8
- Eigen >= 3.3.4
### 2.3 livox_ros_drvier

## 3 Build
```bash
cd $YOUR_CATKIN_WORESPACE$/src
git clone https://github.com/lxh3181515/pointcloud_gravity_align.git
cd ..
catkin_make
```

## 4 Set parameters
In launch file (launch/run.launch):
- lidar_type:
    - 0 - standard pointcloud type (sensor_msgs::pointcloud2)
    - 1 - livox pointcloud type (livox_ros_driver::CustomMsg)
- sub_lidar/imu_topic: raw lidar/imu topic name
- pub_lidar/imu_topic: lidar/imu topic name after aligning

## 5 Run
```bash
source devel/setup.bash
roslaunch pointcloud_gravity_align run.launch
```
