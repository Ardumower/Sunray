
/*
reflector detecion for Livox MID-360 - detect lines in search area: segment lines in LiDAR points

reflector layout:

vertical line1  (0.27m apart from vertical line3 - forms with line3 a plane thus gives tag orientation)
          vertical line2  ( defines tag coordinate system origin)
                   vertical line3
#         #        #    
#         #        #
#         #        #    
#         #        #
#         #        #    


1. cluster segmentation to find the tag with the three lines on it (one cluster)
2. region growing (3 times) to find the points belonging to each line
3. line approximation to find the line model parameters (position and direction)
4. direction vector approximation of the 3 line direction vectors for tag plane orientation estimation
5. center line position is used for tag plane position estimation
6. finally, the position and orientation estimation is low-pass filtered 


*/

#include <ros/package.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>
#include <Eigen/Geometry> 
#include <Eigen/Core>
#include <visualization_msgs/Marker.h>
#include <deque>  // For ring buffer
#include <random>



#define DEBUG false


struct Line {
    Eigen::Vector3f point;  // Ein Punkt auf der Linie
    Eigen::Vector3f direction;  // Richtungsvektor der Linie
};


const int BUFFER_SIZE = 1;  // Number of point clouds to keep

// Declare a ring buffer to hold the last 10 point clouds
std::deque<pcl::PointCloud<pcl::PointXYZI>::Ptr> point_cloud_buffer;


// Struktur zum Speichern der Pose (Position und Orientierung)
struct PoseFilter {
    Eigen::Vector3f filtered_position;
    Eigen::Quaternionf filtered_orientation;
    float alpha;  // Filterkonstante (zwischen 0 und 1)
    bool init; 
    ros::Time last_time;

    PoseFilter() : alpha(0.9f) {  // Beispielwert für den Filter
        filtered_position = Eigen::Vector3f::Zero();
        filtered_orientation = Eigen::Quaternionf::Identity();
        init = false;
    }

    // Aktualisierung der gefilterten Pose (Tiefpassfilter)
    void update(const Eigen::Vector3f& new_position, const Eigen::Quaternionf& new_orientation) {
        if (!init){
            init = true;
            last_time = ros::Time::now();
        }

        // Zeitschritt berechnen
        ros::Time current_time = ros::Time::now();
        float dt = (current_time - last_time).toSec();
        if (dt > 1.0) dt = 1.0;
        if (dt < 0.0001) dt = 0.0001; 
        last_time = current_time;

        // Alpha für dynamische Zeitkonstante
        float alpha_dynamic = alpha * dt;

        // Position filtern
        filtered_position = alpha_dynamic * new_position + (1.0f - alpha_dynamic) * filtered_position;

        // Orientierung filtern
        filtered_orientation = filtered_orientation.slerp(alpha_dynamic, new_orientation);
    }
};



// Hilfsfunktion zum Berechnen des Abstands eines Punktes von einer Linie
float distancePointToLine(const Eigen::Vector3f& point, const Eigen::Vector3f& line_start, const Eigen::Vector3f& line_direction) {
    Eigen::Vector3f line_to_point = point - line_start;
    Eigen::Vector3f projection = line_to_point.dot(line_direction) * line_direction;
    Eigen::Vector3f perpendicular = line_to_point - projection;
    return perpendicular.norm();
}


#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>



void approximateLine(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, pcl::ModelCoefficients::Ptr& line_coefficients) {
    // Berechnung des Mittelpunkts des Clusters (Mittelwert aller Punkte)
    Eigen::Vector3f centroid(0, 0, 0);
    for (const auto& point : cloud->points) {
        centroid += Eigen::Vector3f(point.x, point.y, point.z);
    }
    centroid /= cloud->points.size();

    // Berechnung der Kovarianzmatrix
    Eigen::Matrix3f covariance_matrix = Eigen::Matrix3f::Zero();
    for (const auto& point : cloud->points) {
        Eigen::Vector3f centered_point(point.x - centroid.x(), point.y - centroid.y(), point.z - centroid.z());
        covariance_matrix += centered_point * centered_point.transpose();
    }

    // Eigenwertzerlegung zur Bestimmung der Hauptachse (Richtungsvektor)
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance_matrix);
    Eigen::Vector3f direction = eigen_solver.eigenvectors().col(2);  // Eigenvektor mit größtem Eigenwert (Richtungsvektor)

    // Speichern der Parameter in ModelCoefficients
    line_coefficients->values.resize(6);
    line_coefficients->values[0] = centroid.x();
    line_coefficients->values[1] = centroid.y();
    line_coefficients->values[2] = centroid.z();
    line_coefficients->values[3] = direction.x();
    line_coefficients->values[4] = direction.y();
    line_coefficients->values[5] = direction.z();
}


void regionGrowing(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, float max_distance, std::vector<pcl::PointIndices>& clusters,
                        std::vector<pcl::PointIndices>& line_inliers) {
    // Initialize KdTree for nearest neighbor search
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(cloud);

    // Vector to keep track of which points are already processed
    std::vector<bool> processed(cloud->points.size(), false);

    // Loop through all points in the cloud
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        if (processed[i]) continue;  // Skip already processed points

        // Create a new cluster
        pcl::PointIndices cluster;
        pcl::PointIndices inliers;        
        std::vector<int> seed_queue;  // Queue of points to process
        seed_queue.push_back(i);      // Start with the current point
        processed[i] = true;          // Mark as processed

        // Region-growing loop
        while (!seed_queue.empty()) {
            int current_point = seed_queue.back();
            seed_queue.pop_back();

            // Add current point to the cluster
            cluster.indices.push_back(current_point);
            inliers.indices.push_back(current_point);

            // Perform radius search to find neighbors within max_distance
            std::vector<int> neighbors;
            std::vector<float> distances;
            tree->radiusSearch(cloud->points[current_point], max_distance, neighbors, distances);

            // Loop through neighbors
            for (size_t j = 0; j < neighbors.size(); ++j) {
                if (!processed[neighbors[j]]) {
                    // Mark neighbor as processed and add it to the seed queue
                    processed[neighbors[j]] = true;
                    seed_queue.push_back(neighbors[j]);
                }
            }
        }

        // Add the cluster if it's not too small
        if (cluster.indices.size() >= 5) {  // You can adjust the minimum cluster size
            clusters.push_back(cluster);
            line_inliers.push_back(inliers);
        }
    }
}


// RANSAC für Liniensegmentierung in 3D (mit Ausgabe im pcl::ModelCoefficients Format und Inlier Indizes)
void ransacLineSegmentation(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, float distance_threshold, int max_iterations, pcl::ModelCoefficients::Ptr best_line_coefficients, pcl::PointIndices::Ptr inliers) {
    int best_inlier_count = 0;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, cloud->points.size() - 1);

    // Initialisiere die besten Linienkoeffizienten und Inliers
    best_line_coefficients->values.resize(6);
    inliers->indices.clear();

    // RANSAC-Iterationen
    for (int i = 0; i < max_iterations; ++i) {
        // Zwei zufällige Punkte auswählen
        int idx1 = dis(gen);
        int idx2 = dis(gen);

        Eigen::Vector3f point1(cloud->points[idx1].x, cloud->points[idx1].y, cloud->points[idx1].z);
        Eigen::Vector3f point2(cloud->points[idx2].x, cloud->points[idx2].y, cloud->points[idx2].z);

        // Berechne den Richtungsvektor der Linie
        Eigen::Vector3f line_direction = (point2 - point1).normalized();

        // Zähle die Inliers, also Punkte, die nah genug an der Linie liegen
        int inlier_count = 0;
        std::vector<int> current_inliers;
        for (int j = 0; j < cloud->points.size(); ++j) {
            Eigen::Vector3f p(cloud->points[j].x, cloud->points[j].y, cloud->points[j].z);
            float distance = distancePointToLine(p, point1, line_direction);
            if (distance < distance_threshold) {
                current_inliers.push_back(j);
                inlier_count++;
            }
        }

        // Speichere das beste Linienmodell (mit den meisten Inliers)
        if (inlier_count > best_inlier_count) {
            best_inlier_count = inlier_count;

            // Speichere die Linienkoeffizienten: point1 als Startpunkt und line_direction als Richtungsvektor
            best_line_coefficients->values[0] = point1.x();
            best_line_coefficients->values[1] = point1.y();
            best_line_coefficients->values[2] = point1.z();
            best_line_coefficients->values[3] = line_direction.x(); 
            best_line_coefficients->values[4] = line_direction.y();
            best_line_coefficients->values[5] = line_direction.z();

            // Speichere die Inliers
            inliers->indices = current_inliers;
        }
    }
}



// Function to sum up the point clouds in the buffer
pcl::PointCloud<pcl::PointXYZI>::Ptr sumPointClouds() {
    pcl::PointCloud<pcl::PointXYZI>::Ptr summed_cloud(new pcl::PointCloud<pcl::PointXYZI>());

    for (const auto& cloud : point_cloud_buffer) {
        *summed_cloud += *cloud;  // Add each cloud to the summed cloud
    }

    return summed_cloud;
}

Eigen::Vector3f calculateZAxis(const Eigen::Vector3f& direction_horizontal, const Eigen::Vector3f& point1, const Eigen::Vector3f& point2) {
    // Berechne den Verbindungsvektor von einem Punkt auf der ersten Linie (point1) zum Punkt auf der zweiten Linie (point2)
    Eigen::Vector3f v_connect = point2 - point1;

    // Berechne das Kreuzprodukt des Richtungsvektors der horizontalen Linien und des Verbindungsvektors
    Eigen::Vector3f z_axis = direction_horizontal.cross(v_connect);

    // Normalisiere die Z-Achse, um eine Längeneinheit zu erhalten
    z_axis.normalize();

    return z_axis;
}

// Berechnet den minimalen Abstand zwischen zwei parallelen Linien
float calculateMinimalDistanceBetweenParallelLines(const Eigen::Vector3f& point_on_line1, const Eigen::Vector3f& point_on_line2, const Eigen::Vector3f& direction_vector) {
    // Normalisiere den Richtungsvektor
    Eigen::Vector3f direction = direction_vector.normalized();

    // Berechnung des Vektors zwischen den beiden Punkten
    Eigen::Vector3f diff = point_on_line2 - point_on_line1;

    // Die Distanz zwischen den parallelen Linien ist die Projektion des Vektors diff auf einen Vektor, der senkrecht zur Richtung der Linien steht
    // In diesem Fall verwenden wir das Kreuzprodukt des Richtungsvektors mit der Z-Achse, um die senkrechte Richtung zu erhalten.
    Eigen::Vector3f normal = direction.cross(Eigen::Vector3f::UnitZ());

    // Falls der Kreuzprodukt-Vektor nahe Null ist, verwenden wir einen alternativen Normalenvektor
    if (normal.norm() < 1e-6) {
        normal = direction.cross(Eigen::Vector3f::UnitX());
    }

    normal.normalize(); // Normalisiere den Normalenvektor

    // Berechne den minimalen Abstand
    float distance = std::fabs(diff.dot(normal));

    return distance;
}


bool isBetween(const Eigen::Vector3f& point1, const Eigen::Vector3f& point2, const Eigen::Vector3f& point3, float tolerance) {
    // Berechne die Verbindungsgerade zwischen Punkt 1 und Punkt 3
    Eigen::Vector3f line1_to_line3 = point3 - point1;
    Eigen::Vector3f line1_to_line2 = point2 - point1;

    // Berechne die Projektion von Punkt 2 auf die Verbindungsgerade zwischen Punkt 1 und Punkt 3
    float projection = line1_to_line2.dot(line1_to_line3.normalized());

    // Berechne die Länge der Strecke zwischen Punkt 1 und Punkt 3
    float distance_line1_to_line3 = line1_to_line3.norm();

    // Berechne den Abstand von Punkt 2 zur Verbindungsgerade zwischen Punkt 1 und Punkt 3
    Eigen::Vector3f projected_point = point1 + projection * line1_to_line3.normalized();
    float distance_to_line = (projected_point - point2).norm();

    // Prüfe, ob die Projektion von Punkt 2 in der Nähe zwischen Punkt 1 und Punkt 3 liegt (mit Toleranz)
    bool within_projection = projection > -tolerance && projection < distance_line1_to_line3 + tolerance;

    // Prüfe, ob der Abstand von Punkt 2 zur Linie innerhalb der Toleranz liegt
    bool within_distance = distance_to_line <= tolerance;

    // Punkt 2 muss innerhalb der Projektion und im Toleranzbereich der Linie liegen
    return within_projection && within_distance;
}


void rotateAroundVector(const Eigen::Vector3f& axis, Eigen::Vector3f& v, float angle_in_degrees) {
    // Umwandlung von Grad in Bogenmaß
    float theta = angle_in_degrees * M_PI / 180.0f;

    // Normiere den Achsenvektor
    Eigen::Vector3f k = axis.normalized();

    // Berechne die Rotation mit der Rodrigues-Formel
    v = v * std::cos(theta) + k.cross(v) * std::sin(theta) + k * (k.dot(v)) * (1 - std::cos(theta));
}



class ReflectorDetect
{
public:
    ReflectorDetect()
    {
        ros::NodeHandle nh;

        imu_sub_ = nh.subscribe("/livox/imu_aligned", 10, &ReflectorDetect::imuCallback, this);
        point_cloud_sub_ = nh.subscribe("/livox/lidar_aligned", 1, &ReflectorDetect::pointCloudCallback, this);
        grad_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/reflector/grad_points", 10);        
        line_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/reflector/line_points", 10);
        cluster_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/reflector/cluster_points", 10);
        pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/reflector/pose", 1);
        //lidarAccumulator = new LidarAccumulator(nh, "/livox/lidar_aligned", "livox_frame");
        cube_pub = nh.advertise<visualization_msgs::Marker>("/reflector/cube", 1);
    }

    void publishCubeMarker(const Eigen::Vector3f& position, float x_length, float y_width, float z_height, float angle_deg, const std::string& frame_id) {
        // Erstelle den Marker
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "cube";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;

        // Setze die Position des Würfels
        marker.pose.position.x = position.x();
        marker.pose.position.y = position.y();
        marker.pose.position.z = position.z();

        // Setze die Orientierung (10 Grad Neigung um die X-Achse)
        Eigen::AngleAxisf rotation(angle_deg * M_PI / 180.0f, Eigen::Vector3f::UnitY());
        Eigen::Quaternionf quaternion(rotation);
        marker.pose.orientation.x = quaternion.x();
        marker.pose.orientation.y = quaternion.y();
        marker.pose.orientation.z = quaternion.z();
        marker.pose.orientation.w = quaternion.w();

        // Setze die Dimensionen des Würfels
        marker.scale.x = x_length;
        marker.scale.y = y_width;
        marker.scale.z = z_height;

        // Setze die Farbe auf Weiß
        marker.color.r = 1.0f;
        marker.color.g = 1.0f;
        marker.color.b = 1.0f;
        marker.color.a = 0.2f; // Volle Deckkraft

        // Veröffentliche den Marker
        cube_pub.publish(marker);
    }


    // Funktion zur Berechnung des euklidischen Abstands zwischen zwei Punkten
    double calculateDistance(const pcl::PointXYZI& pt1, const pcl::PointXYZI& pt2) {
        return std::sqrt(std::pow(pt1.x - pt2.x, 2) +
                        std::pow(pt1.y - pt2.y, 2) +
                        std::pow(pt1.z - pt2.z, 2));
    }

    // Funktion zur Berechnung der Länge einer Linie basierend auf den Punkten der Inliers
    double calculateLineLength(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const pcl::PointIndices& inliers) {
        double max_distance = 0.0;
        pcl::PointXYZI pt1, pt2;

        // Über alle Punktpaare iterieren, um den maximalen Abstand zu finden
        for (size_t i = 0; i < inliers.indices.size(); ++i) {
            for (size_t j = i + 1; j < inliers.indices.size(); ++j) {
                pt1 = cloud->points[inliers.indices[i]];
                pt2 = cloud->points[inliers.indices[j]];
                double distance = calculateDistance(pt1, pt2);
                if (distance > max_distance) {
                    max_distance = distance;
                }
            }
        }

        return max_distance;
    }

    // Funktion zur Detektion von Linien mit RANSAC-Segmentierung in einer Punktwolke
    bool detectLinesInCluster(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                            std::vector<pcl::ModelCoefficients::Ptr>& line_coefficients,
                            std::vector<pcl::PointIndices>& line_inliers,
                            int max_lines = 3) {

        if (DEBUG) ROS_WARN("cluster %d", (int)cloud->points.size());                    
        // Region Growing Setup
        std::vector<pcl::PointIndices> clusters;

        regionGrowing(cloud, 0.055, clusters, line_inliers);
        if (DEBUG) ROS_WARN("region clusters %d", (int)clusters.size());

        if (clusters.size() != 3) return false;

        // Iterate over each region/cluster and apply RANSAC line fitting
        for (const auto& cluster : clusters) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr region_cluster(new pcl::PointCloud<pcl::PointXYZI>());

            // Extract points from the current cluster
            for (const auto& idx : cluster.indices) {
                region_cluster->points.push_back(cloud->points[idx]);
            }
            int numPoints = region_cluster->points.size();
            if (DEBUG) ROS_WARN("region_cluster %d", (int)numPoints);        
            if (region_cluster->points.size() < 3) return false;
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            //pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            approximateLine(region_cluster, coefficients);
            /*
                pcl::SACSegmentation<pcl::PointXYZI> seg;
                seg.setModelType(pcl::SACMODEL_LINE);
                seg.setMethodType(pcl::SAC_RANSAC);
                seg.setDistanceThreshold(0.03); // 0.03
                seg.setInputCloud(region_cluster);
                seg.setMaxIterations(100);
                seg.segment(*inliers, *coefficients);

                // Liniensegmentierung mit RANSAC durchführen
                //ransacLineSegmentation(region_cluster, 0.02f, 100, coefficients, inliers);
            */

            //if ( (inliers->indices.size() < ((float)numPoints) * 0.2) || (inliers->indices.size() > ((float)numPoints) * 0.4) )  {
            //if ( inliers->indices.size() < 3)  {            
            //    return false;
            //}
            //if (DEBUG) ROS_WARN("inliers: %d", (int)inliers->indices.size());

            if (coefficients->values[5] < 0){ // direction vector should point upwards
                coefficients->values[3] *= -1;
                coefficients->values[4] *= -1;
                coefficients->values[5] *= -1;
            }
            line_coefficients.push_back(coefficients);            
            //line_inliers.push_back(inliers);
        }        
        //if ( ((float)region_cluster->points.size()) / ((float)numPoints) > 0.2) return false;  // don't expect many remaining points
        return true;
    }


    // Funktion zur Überprüfung, ob die Linien die relative Orientierung des Reflektorkoordinatensystems haben
    bool isReflectorCoordinateSystem(const std::vector<pcl::ModelCoefficients::Ptr>& line_coefficients, 
                                    const std::vector<double>& line_lengths) {        
        if (DEBUG) ROS_INFO("lines %d", (int)line_lengths.size());
        if (line_lengths.size() != 3) {
            if (DEBUG) ROS_WARN("invalid line count: %d", (int)line_lengths.size());
            return false;
        }

        Eigen::Vector3f pt1(line_coefficients[0]->values[0], line_coefficients[0]->values[1], line_coefficients[0]->values[2]);
        Eigen::Vector3f pt2(line_coefficients[1]->values[0], line_coefficients[1]->values[1], line_coefficients[1]->values[2]);
        Eigen::Vector3f pt3(line_coefficients[2]->values[0], line_coefficients[2]->values[1], line_coefficients[2]->values[2]);

        /*if ( (abs(pt1.y() - 0) > 0.8) ||  (abs(pt2.y() - 0) > 0.8) || (abs(pt3.y() - 0) > 0.8) ){
            if (DEBUG) ROS_WARN("line points not in lidar fov");
            return false;
        } */

        if ( !isBetween(pt1, pt2, pt3, 0.1) ) {
            if (DEBUG) ROS_WARN("line2 not in between lines 1,3");
            return false;
        }
    
        // line1,2,3 should be left-to-right
        if ( ! ( (pt2.y() > pt1.y()) && (pt3.y() > pt2.y()) )  ) {
            if (DEBUG) ROS_WARN("line1,2,3 not left-to-right");
            return false;  
        }
    
        // Längenprüfung: zwei horizontale Linien (ca. 45 cm) und eine vertikale Linie (ca. 16 cm)

        bool isLine1Valid = (line_lengths[0] > 0.03) && (line_lengths[0] < 0.8);
        bool isLine2Valid = (line_lengths[1] > 0.03) && (line_lengths[1] < 0.8);
        bool isLine3Valid = (line_lengths[2] > 0.03) && (line_lengths[2] < 0.8);

        if (!(isLine1Valid && isLine2Valid && isLine3Valid)) {
            if (DEBUG) ROS_WARN("invalid lens: %.2f, %.2f, %.2f", line_lengths[0], line_lengths[1], line_lengths[2]);
            return false;
        }
        //if (DEBUG) ROS_WARN("lines ok");
        
        // Winkelprüfung: Linien müssen senkrecht zueinander sein (innerhalb eines Toleranzwinkels)
        // Winkel zwischen den Linien wird anhand der Richtungsvektoren (dx, dy, dz) berechnet
        Eigen::Vector3f dir1(line_coefficients[0]->values[3], line_coefficients[0]->values[4], line_coefficients[0]->values[5]);
        Eigen::Vector3f dir2(line_coefficients[1]->values[3], line_coefficients[1]->values[4], line_coefficients[1]->values[5]);
        Eigen::Vector3f dir3(line_coefficients[2]->values[3], line_coefficients[2]->values[4], line_coefficients[2]->values[5]);

        // Linien 1 und 2 müssen parallel sein
        float dot_product_12 = dir1.dot(dir2);
        bool areParallel1 = std::abs(dot_product_12 - 1.0) < 0.1;

        // Linie 2 und 3 müssen parallel sein
        float dot_product_13 = dir2.dot(dir3);
        bool areParallel2 = std::abs(dot_product_13 - 1.0) < 0.1;

        //areParallel = true;
        //isPerpendicular = true;

        if (! (areParallel1 && areParallel2)){
            if (DEBUG) ROS_WARN("not parallel %.2f, %.2f", std::abs(dot_product_12 - 1.0), std::abs(dot_product_13 - 1.0));
            return false;
        }

        // Berechne den minimalen Abstand zwischen den beiden Linien
        Eigen::Vector3f direction(line_coefficients[0]->values[3], line_coefficients[0]->values[4], line_coefficients[0]->values[5]);
        float distance = calculateMinimalDistanceBetweenParallelLines(pt1, pt3, direction);

        if ((distance < 0.1) || (distance > 0.35)){
            if (DEBUG) ROS_WARN("invalid dist: %.2f", distance);
            return false;            
        }

        //if (DEBUG) ROS_WARN("len %.2f, %.2f, %.2f", line_lengths[0], line_lengths[1], line_lengths[2]);
        //if (DEBUG) ROS_WARN("parallel %.2f, %.2f", std::abs(dot_product_12 - 1.0), std::abs(dot_product_13 - 1.0));
        //if (DEBUG) ROS_WARN("dist %.2f", distance);            
        return true;
    }


    // Funktion zur Veröffentlichung der Reflektor-Pose als tf-Transformation
    void publishReflectorTransform(const Eigen::Vector3f& position, 
                                const Eigen::Quaternionf q, 
                                const std::string& lidar_frame, 
                                const std::string& reflector_frame) {
        // Initialisiere den tf-Transform-Broadcaster
        static tf::TransformBroadcaster br;

        // Erstelle eine tf-Transformation
        tf::Transform transform;

        // Setze die Translation basierend auf der Reflektor-Position
        transform.setOrigin(tf::Vector3(position.x(), position.y(), position.z()));

        // Erstelle die Rotation basierend auf den Richtungsvektoren (x, y, z)
        //tf::Matrix3x3 rotation_matrix(
        //    x_axis.x(), y_axis.x(), z_axis.x(),
        //    x_axis.y(), y_axis.y(), z_axis.y(),
        //    x_axis.z(), y_axis.z(), z_axis.z()
        //);

        tf::Quaternion tq;
        tq.setX( q.x() );
        tq.setY( q.y() );
        tq.setZ( q.z() );
        tq.setW( q.w() );
    
        //rotation_matrix.getRotation(q);
        //q.normalize();
        transform.setRotation(tq);

        // Veröffentliche die Transformation zwischen dem Lidar-Frame und dem Reflektor-Frame
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), lidar_frame, reflector_frame));
    }


    // Funktion zur Publikation des Reflektorkoordinatensystems als Pose-Nachricht
    void publishReflectorPose(ros::Publisher& pub, const pcl::ModelCoefficients::Ptr& line1, 
        const pcl::ModelCoefficients::Ptr& line2, const pcl::ModelCoefficients::Ptr& line3) {
        geometry_msgs::PoseStamped pose;

        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "livox_frame";
            
        Eigen::Vector3f point1(line1->values[0], line1->values[1], line1->values[2]);  // Startpunkt der ersten Linie
        Eigen::Vector3f point3(line3->values[0], line3->values[1], line3->values[2]);  // Startpunkt der zweiten Linie

        //Eigen::Vector3f dir2(line2->values[3], line2->values[4], line2->values[5]);

        // Extrahieren der Richtungsvektoren der drei Linien
        Eigen::Vector3f dir1(line1->values[3], line1->values[4], line1->values[5]);
        Eigen::Vector3f dir2(line2->values[3], line2->values[4], line2->values[5]);
        Eigen::Vector3f dir3(line3->values[3], line3->values[4], line3->values[5]);
        dir1.normalize();
        dir2.normalize();
        dir3.normalize();

        // Mittelung der Richtungsvektoren
        Eigen::Vector3f averaged_dir = (dir1 + dir2 + dir3) / 3.0;

        // Normieren des gemittelten Vektors
        averaged_dir.normalize();
        
        //Eigen::Vector3f x_axis = (-(point3 -point1)).normalized();  // Richtung der X-Achse

        //Eigen::Vector3f z_axis = dir2;

        // Y-Achse: Senkrecht zur X- und Z-Achse
        //Eigen::Vector3f y_axis = z_axis.cross(x_axis).normalized();

        // Rotieren um 180 Grad um den X-Richtungsvektor
        //rotateAroundVector(x_axis, y_axis, 180.0f);  // Rotiert Y um die X-Achse
        //rotateAroundVector(x_axis, z_axis, 180.0f);  // Rotiert Z um die X-Achse
        
        // Berechnung der Quaternion für die Orientierung aus den Achsen
        //Eigen::Matrix3f rotation_matrix;
        //rotation_matrix.col(0) = x_axis; // X-Achse
        //rotation_matrix.col(1) = y_axis; // Y-Achse
        //rotation_matrix.col(2) = z_axis; // Z-Achse

        Eigen::Quaternionf quaternion;
        quaternion.setFromTwoVectors(Eigen::Vector3f(0.0, 0.0, 1.0), averaged_dir);  // Richtungsvektor in Pose-Orientierung umwandeln
    
        Eigen::Vector3f position(line2->values[0], line2->values[1], line2->values[2]);

        pose_filter.update(position, quaternion);

        pose.pose.position.x = pose_filter.filtered_position.x();
        pose.pose.position.y = pose_filter.filtered_position.y();
        pose.pose.position.z = pose_filter.filtered_position.z();
    
        pose.pose.orientation.x = pose_filter.filtered_orientation.x();
        pose.pose.orientation.y = pose_filter.filtered_orientation.y();
        pose.pose.orientation.z = pose_filter.filtered_orientation.z();
        pose.pose.orientation.w = pose_filter.filtered_orientation.w();
        
        pub.publish(pose);

        // Dummy-Daten für Position und Achsen des Reflektors (Normalerweise aus Berechnungen)
        Eigen::Vector3f reflector_position(line2->values[0], line2->values[1], line2->values[2]);

        // LiDAR-Frame und Reflektor-Frame Namen
        std::string lidar_frame = "livox_frame";
        std::string reflector_frame = "reflector_frame";
        publishReflectorTransform(pose_filter.filtered_position, pose_filter.filtered_orientation, lidar_frame, reflector_frame);
    }


    void processCloud(){
        pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_points(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr line_points(new pcl::PointCloud<pcl::PointXYZI>());

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        
        pcl::fromROSMsg(*cloudMsg, *cloud);

        //if (DEBUG) ROS_WARN("processCloud %d", (int)cloud->points.size());
        if (cloud->points.size() == 0) return;

        // Add the new cloud to the buffer
        if (point_cloud_buffer.size() >= BUFFER_SIZE) {
            point_cloud_buffer.pop_front();  // Remove the oldest point cloud
        }
        point_cloud_buffer.push_back(cloud);  // Add the new point cloud

        // Sum up the point clouds in the buffer
        pcl::PointCloud<pcl::PointXYZI>::Ptr accumulated_cloud = sumPointClouds();

        // Filter based on intensity
        pcl::PointCloud<pcl::PointXYZI>::Ptr high_gradient_points(new pcl::PointCloud<pcl::PointXYZI>());
        for (const auto& point : accumulated_cloud->points) {
            if (point.x < 0.0) {  // must be at backside of lidar  
                if (abs(point.y -0) < 0.7) {  // must be in lidar FOV 
                    //if (point.z < 1.0) { 
                        if (point.intensity > 50) {  // Set your intensity threshold here
                            high_gradient_points->points.push_back(point);
                        }
                    //}
                }
            }
        }
                

        // Schritt 1: Clustering der Punktwolke mit Euclidean Cluster Extraction
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
        tree->setInputCloud(high_gradient_points);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance(0.1); // Toleranz für Cluster
        ec.setMinClusterSize(20);
        ec.setMaxClusterSize(400);
        ec.setSearchMethod(tree);
        ec.setInputCloud(high_gradient_points);
        ec.extract(cluster_indices);


        if (DEBUG) ROS_INFO("-----------------------------");
        if (DEBUG) ROS_INFO("cluster %d", (int)cluster_indices.size());

        // Schritt 2: Verarbeitung jedes Clusters zur Liniendetektion
        int clusterCounter = 0;
        bool markerFound = false;
        for (const auto& cluster_idx : cluster_indices) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::copyPointCloud(*high_gradient_points, cluster_idx.indices, *cloud_cluster);
            //if (DEBUG) ROS_INFO("pts %d", (int)cloud_cluster->points.size());
            for (size_t i = 0; i < cloud_cluster->points.size(); ++i) {
                pcl::PointXYZI pt;
                pt = cloud_cluster->points[i];
                pt.intensity = (clusterCounter * 40) % (255-30) + 30;                    
                cluster_points->points.push_back(pt);
            }

            std::vector<pcl::ModelCoefficients::Ptr> line_coefficients;
            std::vector<pcl::PointIndices> line_inliers;

            if (!markerFound){
                // Schritt 3: Detektion von Linien innerhalb des Clusters
                if (detectLinesInCluster(cloud_cluster, line_coefficients, line_inliers)){

                    // Schritt 4: Berechnung der Linienlängen und Speicherung
                    std::vector<double> line_lengths;
                    int lineCounter = 0;
                    if (DEBUG) ROS_WARN("line_inliers %d", (int)line_inliers.size());
                    for (const auto& inliers : line_inliers) {
                        pcl::PointXYZI pt;
                        if (DEBUG) ROS_WARN("inliers %d", (int)inliers.indices.size());
                        for (size_t i = 0; i < inliers.indices.size(); ++i) {
                            pt = cloud_cluster->points[inliers.indices[i]];
                            pt.intensity = (lineCounter * 40) % (255-30) + 30;
                            line_points->points.push_back(pt);                    
                        }
                        double length = calculateLineLength(cloud_cluster, inliers);
                        line_lengths.push_back(length);
                        lineCounter++;
                    }

                    // Schritt 5: Prüfung, ob die Linien das gewünschte Koordinatensystem bilden
                    std::vector<int> perm = {0, 1, 2};
                    // Teste alle Permutationen der drei Linien
                    do {
                        if (markerFound) break;
                        // Rufe isReflectorCoordinateSystem mit der aktuellen Permutation der Linien auf
                        std::vector<pcl::ModelCoefficients::Ptr> line_coefficients_perm;
                        std::vector<double> line_lengths_perm;
                        line_coefficients_perm.push_back(line_coefficients[perm[0]]);
                        line_coefficients_perm.push_back(line_coefficients[perm[1]]);
                        line_coefficients_perm.push_back(line_coefficients[perm[2]]);
                        line_lengths_perm.push_back(line_lengths[perm[0]]);
                        line_lengths_perm.push_back(line_lengths[perm[1]]);
                        line_lengths_perm.push_back(line_lengths[perm[2]]);

                        if (isReflectorCoordinateSystem(line_coefficients_perm, line_lengths_perm)) {
                            // Wenn eine gültige Permutation gefunden wurde, berechne und veröffentliche die Pose
                            publishReflectorPose(pose_pub, 
                                line_coefficients_perm[0], 
                                line_coefficients_perm[1], 
                                line_coefficients_perm[2]);
                            markerFound = true;
                            break;  // Beende die Funktion, da die Pose bereits veröffentlicht wurde
                        }
                    } while (std::next_permutation(perm.begin(), perm.end()));  // Erzeuge die nächste Permutation
                }
            }
            clusterCounter++;
        }


        sensor_msgs::PointCloud2 line_msg;
        pcl::toROSMsg(*line_points, line_msg);
        line_msg.header = cloudMsg->header;
        line_pub_.publish(line_msg);

        sensor_msgs::PointCloud2 grad_msg;
        pcl::toROSMsg(*high_gradient_points, grad_msg);
        grad_msg.header = cloudMsg->header;
        grad_pub_.publish(grad_msg);

        sensor_msgs::PointCloud2 cluster_msg;
        pcl::toROSMsg(*cluster_points, cluster_msg);
        cluster_msg.header = cloudMsg->header;
        cluster_pub_.publish(cluster_msg);

        Eigen::Vector3f cube_position(0.3f, 0.0f, 0.0f);  // Im LiDAR-Frame
        publishCubeMarker(cube_position, 1.1, 0.8, 0.2, 0, "base_link");
    }

    // -------------------------------------------------------------------------
    

    bool cloudReceived = false;

private:
    void imuCallback(sensor_msgs::Imu msg) {            
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
    {
        static int cloudCounter = 0;
        cloudCounter++;
        //printf("pointCloudCallback dense=%d height=%d\n", (int)msg->is_dense, (int)msg->height);
        if (cloudReceived) return;
        cloudMsg = msg;
        if (DEBUG) ROS_INFO("cloud %d", cloudCounter);
        cloudReceived = true;
    }


    PoseFilter pose_filter;
    ros::Subscriber point_cloud_sub_;
    ros::Subscriber imu_sub_;
    ros::Publisher line_pub_;
    ros::Publisher cluster_pub_;
    ros::Publisher grad_pub_;
    ros::Publisher pose_pub;
    ros::Publisher cube_pub;
    sensor_msgs::PointCloud2ConstPtr cloudMsg;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "reflector_detect");
    ReflectorDetect detect;
    ros::Rate *rate = new ros::Rate(10);

    while (ros::ok()) {
        // https://stackoverflow.com/questions/23227024/difference-between-spin-and-rate-sleep-in-ros
        //ros::spin();
        ros::spinOnce();
        rate->sleep();
        if (detect.cloudReceived) {
            //if (DEBUG) ROS_INFO("processCloud");
            detect.processCloud();            
            detect.cloudReceived = false;            
        }
        //printf("loop\n");
    }
    return 0;
}
