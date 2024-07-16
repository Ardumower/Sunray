#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>

class GroundLidarProcessor
{
public:
    GroundLidarProcessor()
    {
        ros::NodeHandle nh("~");

        nh.param("angle_opening", angle_opening_, 180.0);
        nh.param("max_distance", max_distance_, 1.0);
        nh.param("max_width", max_width_, 1.5);        
        nh.param("max_height", max_height_, 0.5);        
        nh.param("near_distance", near_distance_, 0.5);
        nh.param("ground_height", ground_height_, 0.0);
        nh.param("min_obstacle_size", min_obstacle_size_, 0.1);
        nh.param("lidar_tilt_angle", lidar_tilt_angle_, 15.0); // Neigungswinkel in Grad

        point_cloud_sub_ = nh.subscribe("/livox/lidar", 1, &GroundLidarProcessor::pointCloudCallback, this);
        ground_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/ground_points", 10);
        obstacle_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/obstacle_points", 10);
        soundTimeout = 0;
        obstacleFar = false;
        obstacleNear = false;
        pkg_loc = ros::package::getPath( ros::this_node::getName().substr(1) );
        printf("pkg_loc: %s\n", pkg_loc.c_str());                
    }

private:
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*msg, *cloud);

        pcl::PointCloud<pcl::PointXYZI>::Ptr ground_points(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr obstacle_points(new pcl::PointCloud<pcl::PointXYZI>());

        // Umwandlung des Neigungswinkels in Radiant
        double tilt_angle_rad = lidar_tilt_angle_ * M_PI / 180.0;

        for (const auto &point : cloud->points)
        {
            double angle = std::atan2(point.y, point.x);
            if (std::abs(angle) > (angle_opening_ / 2.0 * M_PI / 180.0))
                continue;

            // Berechnung der X- und Z-Koordinaten relativ zur Bodenebene
            double adjusted_x = point.x * std::cos(tilt_angle_rad) + point.z * std::sin(tilt_angle_rad);
            double adjusted_y = point.y; // Die Y-Komponente bleibt unverändert
            // Berechnung der Höhe relativ zur Bodenebene
            double adjusted_z = point.z * std::cos(tilt_angle_rad) - point.x * std::sin(tilt_angle_rad);

            if (adjusted_x > max_distance_) continue;            
            if (abs(adjusted_y) > max_width_/2) continue;
            if (adjusted_z > max_height_) continue;

            //if (std::abs(adjusted_z - ground_height_) < 0.05)
            if (adjusted_z < ground_height_ -  0.05)            
            {
                ground_points->points.push_back(point);
            }
            else
            {
                if (adjusted_z > ground_height_ && isObstacle(adjusted_x, adjusted_y, adjusted_z, *cloud))
                {
                    obstacle_points->points.push_back(point);
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

        obstacleFar = false;
        if (obstacle_points->points.size() > 0){
            obstacleFar = true;
        }
        obstacleNear = false;        
        for (const auto &point : obstacle_points->points)
        {
            double adjusted_x = point.x * std::cos(tilt_angle_rad) + point.z * std::sin(tilt_angle_rad);
            if (adjusted_x < near_distance_) obstacleNear = true;            
        }

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
    }

    bool isObstacle(double adjusted_x, double adjusted_y, double adjusted_z, 
        const pcl::PointCloud<pcl::PointXYZI> &cloud)
    {
        int count = 0;

        for (const auto &pt : cloud.points)
        {
            double pt_adjusted_x = pt.x * std::cos(lidar_tilt_angle_ * M_PI / 180.0) + pt.z * std::sin(lidar_tilt_angle_ * M_PI / 180.0);
            double pt_adjusted_z = pt.z * std::cos(lidar_tilt_angle_ * M_PI / 180.0) - pt.x * std::sin(lidar_tilt_angle_ * M_PI / 180.0);
            double pt_adjusted_y = pt.y; // Die Y-Komponente bleibt unverändert

            if (std::abs(pt_adjusted_x - adjusted_x) < min_obstacle_size_ &&
                std::abs(pt_adjusted_y - adjusted_y) < min_obstacle_size_ &&
                std::abs(pt_adjusted_z - adjusted_z) < min_obstacle_size_)
            {
                count++;
            }
        }

        return count > 5;
    }

    ros::Subscriber point_cloud_sub_;
    ros::Publisher ground_pub_;
    ros::Publisher obstacle_pub_;

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
