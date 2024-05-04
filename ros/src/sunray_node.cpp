// run Sunray-based robot as an ROS node 

// start like this:
//
// 1. sudo bash
// 2. source devel/setup.bash
// 3. roslaunch sunray_node test.launch


// http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
// http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20listener%20%28C%2B%2B%29

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

#include "config.h"  
#include "robot.h"



char **argv = NULL;
int argc = 0;


ros::NodeHandle *node;
ros::Rate *rate;
tf::TransformListener *tfListener;
double nextErrorTime = 0;
double nextPrintTime = 0;


void setup(){  
 
  start();

  // Initialize the node
  ros::init(argc, argv, "sunray_node");
  ros::Time::init();

  node = new ros::NodeHandle; 
  tfListener = new tf::TransformListener;

  // Loop at 20Hz, until we shut down
  ROS_INFO("--------------started sunray_node-------------");

  rate = new ros::Rate(50);

} 



void loop(){      
    run(); 

    if (ros::ok()) {
    double tim = ros::Time::now().toSec(); 

    tf::StampedTransform transform;
    try{
        //  http://wiki.ros.org/tf/Tutorials/Time%20travel%20with%20tf%20%28C%2B%2B%29
        tfListener->lookupTransform("map", "gps_link",  ros::Time(0), transform); // target_frame, source_frame
    }
    catch (tf::TransformException ex){
        if (tim > nextErrorTime){
          nextErrorTime = tim + 10.0;
          ROS_ERROR("%s",ex.what());
          //ros::Duration(0.2).sleep();
        }
    }

    if (tim > nextPrintTime){
      nextPrintTime = tim + 10.0;
      float x = transform.getOrigin().x();
      float y = transform.getOrigin().y();
      float z = transform.getOrigin().z();    
      ROS_INFO("x=%.2f  y=%.2f  z=%.2f", x, y, z);
    } 

    // https://stackoverflow.com/questions/23227024/difference-between-spin-and-rate-sleep-in-ros
    //rate->sleep();
    ros::spinOnce();
  } 
  else {
    ROS_ERROR("ROS shutdown");
    //ros::Duration(0.2).sleep();
    exit(0);
  }
}




