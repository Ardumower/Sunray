
/*
ground_lidar_processor ('LiDAR bumper') for Livox MID-360
1. detect ground in search area: segment plane in LiDAR points (with IMU gravity constraint)
   NOTE: Livox LiDAR and Livox IMU share the same coordinate frame, so the IMU gravity vector 
   gives a good initial estimate for the ground plane normal vector  
2. detect obstacles on ground plane in search area
3. trigger bumper event if detected obstacles
*/

#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Int8.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <visualization_msgs/Marker.h>


class GroundLidarProcessor
{
public:
    GroundLidarProcessor()
    {
        ros::NodeHandle nh;

        nh.param("ground_lidar_angle_opening", angle_opening_, 180.0);
        nh.param("ground_lidar_max_distance", max_distance_, 1.0);
        nh.param("ground_lidar_max_width", max_width_, 1.5);        
        nh.param("ground_lidar_max_height", max_height_, 1.0);        
        nh.param("ground_lidar_near_height", near_height_, 0.5);        
        nh.param("ground_lidar_near_distance", near_distance_, 0.5);
        nh.param("ground_lidar_ground_height", ground_height_, -0.3);
        nh.param("ground_lidar_min_obstacle_size", min_obstacle_size_, 0.2);
        nh.param("ground_lidar_tilt_angle", lidar_tilt_angle_, 25.0); // Neigungswinkel in Grad

        imu_sub_ = nh.subscribe("/livox/imu_aligned", 10, &GroundLidarProcessor::imuCallback, this);
        point_cloud_sub_ = nh.subscribe("/livox/lidar_aligned", 1, &GroundLidarProcessor::pointCloudCallback, this);
        ground_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/ground_points", 10);
        obstacle_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/obstacle_points", 10);
        obstacle_state_pub_ = nh.advertise<std_msgs::Int8>("/obstacle_state", 10);
        ground_normal_pub_ = nh.advertise<visualization_msgs::Marker>("/ground_normal", 1);
        imu_gravity_pub_ = nh.advertise<visualization_msgs::Marker>("/imu_gravity", 1);
        printf("angle_opening: %.2f, max_distance: %.2f, max_width: %.2f, max_height: %.2f\n", angle_opening_, max_distance_, max_width_, max_height_);                

        acc_avg = Eigen::Vector3f(0, 0, 0);
        soundTimeout = 0;
        obstacleFar = false;
        obstacleNear = false;
        cloudReceived = false;
        pkg_loc = ros::package::getPath( ros::this_node::getName().substr(1) );
        //ROS_WARN("pkg_loc: %s\n", pkg_loc.c_str());                
    }


    void publishNormalVectorMarker(const Eigen::Vector3f& normal, const Eigen::Vector3f& centroid, ros::Publisher& marker_pub, 
        int red, int green, int blue) {
        // Marker Nachricht erstellen
        visualization_msgs::Marker marker;
        marker.header.frame_id = "livox_frame";  // Anpassen an das verwendete Frame
        marker.header.stamp = ros::Time::now();
        marker.ns = "plane_normal";
        marker.action = visualization_msgs::Marker::ARROW;
        marker.type = visualization_msgs::Marker::ADD;
        marker.id = 0;

        // Orientierung des Pfeils (entspricht dem Normalvektor)
        Eigen::Quaternionf orientation = Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f::UnitX(), normal);
        marker.pose.orientation.x = orientation.x();
        marker.pose.orientation.y = orientation.y();
        marker.pose.orientation.z = orientation.z();
        marker.pose.orientation.w = orientation.w();

        // Skalierung des Pfeils
        marker.scale.x = 3.0;  // Länge des Pfeils
        marker.scale.y = 0.3;  // Breite des Pfeils
        marker.scale.z = 0.3;  // Dicke des Pfeils

        // Farbe des Pfeils
        marker.color.r = red;
        marker.color.g = green;
        marker.color.b = blue;
        marker.color.a = 1.0;
        
        // Veroeffentlichen des Markers
        marker_pub.publish(marker);
    }


    void publishObstacleState(int ground_points, int obstacle_points){
        if ((obstacleFar) || (obstacleNear)) {
            if (soundTimeout == 0){
                ROS_WARN("obstacle_points: %d  ground_points: %d  far %d, near %d", 
                    obstacle_points, ground_points,
                    (int)obstacleFar, (int)obstacleNear );                        

                //std::string command = "../ros/scripts/dbus_send.sh -m Play -p ";
                //std::string command = "mplayer -volume 100 -af volume=5:1 ";                
                //command += pkg_loc; 
                //if (obstacleNear){
                //    command += "/launch/tada.mp3";                
                //} else {
                //    command += "/launch/beep.mp3";                
                //}
                //ROS_WARN("%s", command.c_str());
                //system(command.c_str());
                soundTimeout = 5;                
            }
        }
        if (soundTimeout > 0) soundTimeout--;

        std_msgs::Int8 obstMsg;
        if (obstacleNear) obstMsg.data = 2;
          else if (obstacleFar) obstMsg.data = 1;
          else obstMsg.data = 0;
        obstacle_state_pub_.publish(obstMsg);
    }


    void processCloudNew(){
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*cloudMsg, *cloud);

        // -------- filter-out points above above Z = 1m -------------------------------------------------
        for (int i=0; i < cloud->points.size(); i++)
        {
            auto &pt = cloud->points[i];
            if (abs(pt.x) > 5.0) continue;
            if (abs(pt.y) > 5.0) continue;
            if (pt.x < 0.1) continue; // filter out back-side of LiDAR            
            if (abs(pt.z) > 2.0) continue;
            cloudFiltered->points.push_back(pt);
        }
        
        if (cloudFiltered->size() == 0) {
            ROS_WARN("empty cloudFiltered");     
            return;
        }        

        // ------------- Plane segmentation to find the ground plane  ----------------------------------       
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);        
        /*pcl::SACSegmentation<pcl::PointXYZI> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.05);
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);*/
        pcl::SACSegmentation<pcl::PointXYZI> seg;
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (500);
        float ground_dist_thres = 0.1;
        seg.setDistanceThreshold (ground_dist_thres);    
        //because we want a specific plane (X-Y Plane) (In camera coordinates the ground plane is perpendicular to the z axis)
        //Eigen::Vector3f axis = Eigen::Vector3f(0,0.0,1.0); //z axis
        Eigen::Vector3f axis = gravity_vector_; //z axis        
        seg.setAxis(axis);
        float ground_max_angle = 20.0f;
        seg.setEpsAngle(  ground_max_angle /180.0f * 3.1415f ); // plane can be within 30 degrees of X-Y plane
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloudFiltered);
        seg.segment (*inliers, *coefficients);

        if (inliers->indices.empty())
        {
            ROS_WARN("Could not estimate a planar model for the given dataset.");
            return;
        }

        // ----------- determine plane normal -----------------------------------------------
        Eigen::Vector3f plane_normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
        if (plane_normal(2) < 0) {            
            plane_normal *= -1.0;
        }
        //printf("plane normal %.2f,%.2f,%.2f len: %.2f\n", plane_normal(0), plane_normal(1), plane_normal(2), plane_normal.norm() );            
        float d = coefficients->values[3]; // Distance from origin to the plane                
        //ROS_WARN("d=%.2f\n", d);
        
        //plane_normal.normalize();
        //plane_normal = plane_normal.transpose() ;                
        float angle = std::acos(gravity_vector_.dot(plane_normal));

        /*if (std::abs(angle) > M_PI / 4.0) // Check if the angle is within a reasonable range
        {
            ROS_WARN("The detected plane is not parallel to the ground.");
            return;
        }
        */

        // ------------- Extracting the ground points ------------------------------------------------------
        pcl::PointCloud<pcl::PointXYZI>::Ptr ground_seg_points(new pcl::PointCloud<pcl::PointXYZI>());        
        pcl::ExtractIndices<pcl::PointXYZI> extract;
        extract.setInputCloud(cloudFiltered);
        extract.setIndices(inliers);
        extract.filter(*ground_seg_points);

        // ------------Transform the cloud to align with the ground plane ----------------------------------------
        Eigen::Vector3f z_axis(0, 0, 1);
        Eigen::Quaternionf rotation = Eigen::Quaternionf::FromTwoVectors(z_axis, plane_normal);
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.rotate(rotation);

        // Apply translation based on plane offset
        Eigen::Vector3f translation = -plane_normal * d;
        transform.translate(translation);


        Eigen::Vector3f centroid(translation(0),translation(1),translation(2));        
        publishNormalVectorMarker(plane_normal, centroid, ground_normal_pub_, 0, 1, 1);


        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::transformPointCloud(*cloud, *transformed_cloud, transform.inverse());
        
        // -------------- Extract obstacle points based on the transformed point cloud --------------------------------------
        pcl::PointCloud<pcl::PointXYZI>::Ptr obstacle_points(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr ground_points(new pcl::PointCloud<pcl::PointXYZI>());

        obstacleNear = false;
        obstacleFar = false;

        for (const auto &pt : transformed_cloud->points)
        {
            double distance_to_center = std::sqrt(pt.x * pt.x + pt.y * pt.y);
            double angle = std::atan2(pt.y, pt.x) * 180.0 / M_PI;

            if (false){
                obstacle_points->points.push_back(pt);
            } 
            else {
                ground_points->points.push_back(pt);
                
                if ( (std::abs(pt.x) < 0.05) || (std::abs(pt.y) < 0.05) || (std::abs(pt.z) < 0.05) ) continue;  
                if (pt.x < 0) continue;  

                if (std::abs(angle) > angle_opening_ / 2.0)
                    continue;

                if (pt.x > max_distance_) continue;            
                if (std::abs(pt.y) > max_width_/2) continue;
                if (pt.z > max_height_) continue;

                if (pt.z <  min_obstacle_size_)            
                {
                    //ground_points->points.push_back(pt);
                }            
                else
                {
                    {
                        //if (isObstacle(pt.x, pt.y, pt.z, *transformed_cloud, 0.05))
                        {
                            obstacle_points->points.push_back(pt);
                            obstacleFar = true;                                                                                                       
                            //ROS_INFO("obstacle x=%.2f y=%.2f z=%.2f", pt.x, pt.y, pt.z);    
                            if (pt.x < near_distance_) {
                                if (pt.z < near_height_)  {
                                    //ROS_INFO("obstacleNear x=%.2f y=%.2f z=%.2f", pt.x, pt.y, pt.z);    
                                    obstacleNear = true;
                                }
                            }
                        }
                    }
                }
            }
        }
        if (obstacle_points->points.size() < 60){ //  30 probably false positives
            obstacleNear = false;
            obstacleFar = false;
        }            

        sensor_msgs::PointCloud2 ground_msg;
        pcl::toROSMsg(*ground_points, ground_msg);
        ground_msg.header = cloudMsg->header;
        ground_pub_.publish(ground_msg);

        sensor_msgs::PointCloud2 obstacle_msg;
        pcl::toROSMsg(*obstacle_points, obstacle_msg);
        obstacle_msg.header = cloudMsg->header;
        obstacle_pub_.publish(obstacle_msg);

        publishObstacleState((int)ground_points->points.size(), (int)obstacle_points->points.size()); 

    }

    // -------------------------------------------------------------------------
    
    void processCloudOld(){
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloudAdjusted(new pcl::PointCloud<pcl::PointXYZI>());        
        pcl::fromROSMsg(*cloudMsg, *cloud);
        pcl::fromROSMsg(*cloudMsg, *cloudAdjusted);

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
            ptAdjusted.z = z - ground_height_;
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
            double angle = std::atan2(ptAdjusted.y, ptAdjusted.x);
            if ( (std::abs(pt.x) < 0.05) || (std::abs(pt.y) < 0.05) || (std::abs(pt.z) < 0.05) ) continue;  
            if (std::abs(angle) > (angle_opening_ / 2.0 * M_PI / 180.0))
                continue;

            if (ptAdjusted.x > max_distance_) continue;            
            if (std::abs(ptAdjusted.y) > max_width_/2) continue;
            if (ptAdjusted.z > max_height_) continue;

            //if (std::abs(ptAdjusted.z - ground_height_) < 0.05)
            if (ptAdjusted.z <  min_obstacle_size_)            
            {
                ground_points->points.push_back(ptAdjusted);
            }            
            else
            {
                if (!obstacleNear) {
                    if (isObstacle(pt.x, pt.y, pt.z, *cloud, min_obstacle_size_))
                    {                    
                        obstacle_points->points.push_back(ptAdjusted);   
                        obstacleFar = true;
                        //ROS_INFO("obstacle x=%.2f y=%.2f z=%.2f", pt.x, pt.y, pt.z);    
                        if (ptAdjusted.x < near_distance_) {
                            if (ptAdjusted.z < near_height_)  obstacleNear = true;
                        }                                        
                    }
                }
            }                      
        }

        sensor_msgs::PointCloud2 ground_msg;
        pcl::toROSMsg(*ground_points, ground_msg);
        ground_msg.header = cloudMsg->header;
        ground_pub_.publish(ground_msg);

        sensor_msgs::PointCloud2 obstacle_msg;
        pcl::toROSMsg(*obstacle_points, obstacle_msg);
        obstacle_msg.header = cloudMsg->header;
        obstacle_pub_.publish(obstacle_msg);

        publishObstacleState((int)ground_points->points.size(), (int)obstacle_points->points.size()); 

        //ROS_INFO("pointCloudCallback end");        
    }
    

    bool cloudReceived;

private:
    void imuCallback(sensor_msgs::Imu msg) {        
        float lp = 0.005;
        if (acc_avg.norm() < 0.5) lp = 0.1;        
        acc_avg(0) = (1.0-lp) * acc_avg(0) + lp * msg.linear_acceleration.x;
        acc_avg(1) = (1.0-lp) * acc_avg(1) + lp * msg.linear_acceleration.y;
        acc_avg(2) = (1.0-lp) * acc_avg(2) + lp * msg.linear_acceleration.z;

        // gravity vector should point away from earth, e.g. (0,0,1) if perpendicular to XY-plane 
        //Eigen::Vector3f grav = Eigen::Vector3f(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
        //if (abs(grav.norm() - 1.0) > 0.1) return;
        //printf("g: %.2f,%.2f,%.2f len: %.2f\n", grav(0), grav(1), grav(2), grav.norm());        
        gravity_vector_ = acc_avg; //grav; 
        gravity_vector_.normalize();

        Eigen::Vector3f centroid(0,0,0);        
        publishNormalVectorMarker(gravity_vector_, centroid, imu_gravity_pub_, 1, 0, 1);

        /*         
        Eigen::Vector3f G(0, 0, acc_avg.norm());
        Eigen::Quaternionf q = Eigen::Quaternionf::FromTwoVectors(acc_avg, G);
        Eigen::Matrix3f R_bw = q.toRotationMatrix();
        Eigen::Vector3f eulerAngle = R_bw.eulerAngles(2,1,0);
        //printf("Gravity: %.2fg\n", G(2));
        //std::cout << "RPY Euler angle (rad):\n" << eulerAngle << std::endl;                
        float tilt = eulerAngle(1) / 3.1415 * 180.0;
        if (tilt < 45){
            lidar_tilt_angle_ = tilt; 
            //printf("lidar_tilt_angle=%.2f\n", lidar_tilt_angle_);
        }
        */
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
    {
        //printf("pointCloudCallback\n");
        if (cloudReceived) return;
        cloudMsg = msg;
        //ROS_INFO("pointCloudCallback begin");
        cloudReceived = true;
    }


    bool isObstacle(double x, double y, double z, 
        const pcl::PointCloud<pcl::PointXYZI> &cloud, float min_obstacle_size)
    {
        int count = 0;

        for (const auto &pt : cloud.points)
        {
            if (std::abs(pt.x - x) < min_obstacle_size &&
                std::abs(pt.y - y) < min_obstacle_size &&
                std::abs(pt.z - z) < min_obstacle_size)
            {
                count++;
                if (count > 5) return true;
            }
        }

        return false;
        //return count > 5;
    }

    ros::Subscriber point_cloud_sub_;
    ros::Subscriber imu_sub_;
    ros::Publisher ground_pub_;
    ros::Publisher ground_normal_pub_;
    ros::Publisher imu_gravity_pub_;    
    ros::Publisher obstacle_pub_;
    ros::Publisher obstacle_state_pub_;
    sensor_msgs::PointCloud2ConstPtr cloudMsg;

    Eigen::Vector3f acc_avg;
    Eigen::Vector3f gravity_vector_;
    bool obstacleNear;
    bool obstacleFar;
    double angle_opening_;
    double max_distance_;
    double max_width_;
    double max_height_;    
    double near_distance_;
    double near_height_;        
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
    ros::Rate *rate = new ros::Rate(5);

    while (ros::ok()) {
        // https://stackoverflow.com/questions/23227024/difference-between-spin-and-rate-sleep-in-ros
        //ros::spin();
        ros::spinOnce();
        rate->sleep();
        if (processor.cloudReceived) {
            //processor.processCloudOld();
            processor.processCloudNew();            
            processor.cloudReceived = false;            
        }
        //printf("loop\n");
    }
    return 0;
}
