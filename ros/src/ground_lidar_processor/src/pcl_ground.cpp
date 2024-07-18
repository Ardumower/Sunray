
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Int8.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>

#include "config.h"
  


class GroundLidarProcessor
{
public:
    GroundLidarProcessor()
    {
        ros::NodeHandle nh("~");

        nh.param("angle_opening", angle_opening_, LIDAR_BUMPER_ANGLE_OPENING);
        nh.param("max_distance", max_distance_, LIDAR_BUMPER_MAX_DISTANCE);
        nh.param("max_width", max_width_, LIDAR_BUMPER_MAX_WIDTH);        
        nh.param("max_height", max_height_, LIDAR_BUMPER_MAX_HEIGHT);        
        nh.param("near_distance", near_distance_, LIDAR_BUMPER_NEAR_DISTANCE);
        nh.param("ground_height", ground_height_, LIDAR_BUMPER_GROUND_HEIGHT);
        nh.param("min_obstacle_size", min_obstacle_size_, LIDAR_BUMPER_MIN_OBSTACLE_SIZE);
        nh.param("lidar_tilt_angle", lidar_tilt_angle_, LIDAR_BUMPER_TILT_ANGLE); // Neigungswinkel in Grad
    
        point_cloud_sub_ = nh.subscribe("/livox/lidar", 1, &GroundLidarProcessor::pointCloudCallback, this);
        ground_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/ground_points", 10);
        obstacle_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/obstacle_points", 10);
        obstacle_state_pub_ = nh.advertise<std_msgs::Int8>("/obstacle_state", 10);

        soundTimeout = 0;
        obstacleFar = false;
        obstacleNear = false;
        pkg_loc = ros::package::getPath( ros::this_node::getName().substr(1) );
        printf("pkg_loc: %s\n", pkg_loc.c_str());                
    }

private:
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
    {
        ROS_INFO("pointCloudCallback begin");
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloudAdjusted(new pcl::PointCloud<pcl::PointXYZI>());        
        pcl::fromROSMsg(*msg, *cloud);
        pcl::fromROSMsg(*msg, *cloudAdjusted);

        // Umwandlung des Neigungswinkels in Radiant
        double tilt_angle_rad = lidar_tilt_angle_ * M_PI / 180.0;

        for (int i=0; i < cloud->points.size(); i++)
        {
            auto &pt = cloud->points[i];
            auto &ptAdjusted = cloudAdjusted->points[i];
            double x = pt.x * std::cos(tilt_angle_rad) + pt.z * std::sin(tilt_angle_rad);
            double y = pt.y; // Die Y-Komponente bleibt unverändert
            double z = pt.z * std::cos(tilt_angle_rad) - pt.x * std::sin(tilt_angle_rad);            
            ptAdjusted.x = x;
            ptAdjusted.y = y;
            ptAdjusted.z = z;
            cloudAdjusted->points[i] = ptAdjusted;
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr ground_points(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr obstacle_points(new pcl::PointCloud<pcl::PointXYZI>());

        obstacleFar = false;
        obstacleNear = false;        

        for (int i=0; i < cloud->points.size(); i++)
        {
            auto &pt = cloud->points[i];
            auto &ptAdjusted = cloudAdjusted->points[i];
            double angle = std::atan2(pt.y, pt.x);
            if (std::abs(angle) > (angle_opening_ / 2.0 * M_PI / 180.0))
                continue;

            if (ptAdjusted.x > max_distance_) continue;            
            if (std::abs(ptAdjusted.y) > max_width_/2) continue;
            if (ptAdjusted.z > max_height_) continue;

            //if (std::abs(ptAdjusted.z - ground_height_) < 0.05)
            if (ptAdjusted.z < ground_height_ -  0.05)            
            {
                ground_points->points.push_back(pt);
            }
            else
            {
                if (!obstacleNear) {
                    if (ptAdjusted.z > ground_height_ && isObstacle(ptAdjusted.x, ptAdjusted.y, ptAdjusted.z, *cloud))
                    {                    
                        obstacle_points->points.push_back(pt);   
                        obstacleFar = true;
                        if (pt.x < near_distance_) obstacleNear = true;                                     
                    }
                }
            }                        
        }

        sensor_msgs::PointCloud2 ground_msg;
        pcl::toROSMsg(*ground_points, ground_msg);
        ground_msg.header = msg->header;
        ground_pub_.publish(ground_msg);

        sensor_msgs::PointCloud2 obstacle_msg;
        pcl::toROSMsg(*obstacle_points, obstacle_msg);
        obstacle_msg.header = msg->header;
        obstacle_pub_.publish(obstacle_msg);


        ROS_INFO("obstacle_points: %d  ground_points: %d  far %d, near %d", 
            (int)obstacle_points->points.size(), (int)ground_points->points.size(), 
            (int)obstacleFar, (int)obstacleNear );                        


        if (soundTimeout == 0){
            if ((obstacleFar) || (obstacleNear)) {
                std::string command = "killall mplayer; mplayer -volume 100 -af volume=5:1 ";
                command += pkg_loc; 
                if (obstacleNear){
                    command += "/launch/bell.mp3";                
                } else {
                    command += "/launch/beep.mp3";                
                }
                command += " > /dev/null 2>&1 &";
                system(command.c_str());
                soundTimeout = 5;
            }
        }
        if (soundTimeout > 0) soundTimeout--;

        std_msgs::Int8 obstMsg;
        if (obstacleNear) obstMsg.data = 2;
          else if (obstacleFar) obstMsg.data = 1;
          else obstMsg.data = 0;
        obstacle_state_pub_.publish(obstMsg);
        ROS_INFO("pointCloudCallback end");        
    }


    bool isObstacle(double x, double y, double z, 
        const pcl::PointCloud<pcl::PointXYZI> &cloud)
    {
        int count = 0;

        for (const auto &pt : cloud.points)
        {
            if (std::abs(pt.x - x) < min_obstacle_size_ &&
                std::abs(pt.y - y) < min_obstacle_size_ &&
                std::abs(pt.z - z) < min_obstacle_size_)
            {
                count++;
                if (count > 5) return true;
            }
        }

        return false;
        //return count > 5;
    }

    ros::Subscriber point_cloud_sub_;
    ros::Publisher ground_pub_;
    ros::Publisher obstacle_pub_;
    ros::Publisher obstacle_state_pub_;

    bool obstacleNear;
    bool obstacleFar;
    double angle_opening_;
    double max_distance_;
    double max_width_;
    double max_height_;    
    double near_distance_;    
    double ground_height_;
    double min_obstacle_size_;
    double lidar_tilt_angle_;
    int soundTimeout;
    std::string pkg_loc; 
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ground_lidar_processor");
    GroundLidarProcessor processor;
    ros::spin();
    return 0;
}
