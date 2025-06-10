/**
* @file process_lidar.cpp
* @brief Source (Definition / Implementation) file for the node
* @author Siddhesh Phadke
*/

#include "perception_winter/process_lidar.hpp"
#include <string>
#include <algorithm>
#include <cmath>
#include <open3d/Open3D.h>
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>


void eraseIndices(
    const std::shared_ptr<sensor_msgs::msg::PointCloud>& msg,
    const std::vector<size_t>& indices_to_erase,
    std::vector<std::vector<double>> &positions,
    std::vector<double> &intens
) {
    std::unordered_set<size_t> erase_set(indices_to_erase.begin(), indices_to_erase.end());
    for (size_t i = 0; i < msg->points.size(); ++i) {
        if (erase_set.find(i) == erase_set.end()) {
            const auto& pt = msg->points.at(i);
            positions.push_back({static_cast<double>(pt.x), static_cast<double>(pt.y), static_cast<double>(pt.z)});
            intens.push_back(msg->channels.at(0).values.at(i));
        }
    }
}



ProcessLidar :: ProcessLidar() : Node("process_lidar"){
// Shout out
    RCLCPP_INFO(this->get_logger(), "Process Lidar Node started");

// Initializing
    this->lidar_raw_input_sub = this->create_subscription<sensor_msgs::msg::PointCloud>(
        this->lidar_raw_input_topic,
        10,
        std::bind(&::ProcessLidar::lidar_raw_sub_callback, this, std::placeholders::_1));
    // this->lidar_raw_output_rviz_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    //     this->lidar_raw_output_rviz_topic,
    //     10
    // );
    // // For the reference vehicle
    // this->reference_vehicle_rviz_pub = this->create_publisher<visualization_msgs::msg::Marker>(
    //     this->reference_vehicle_rviz_topic,
    //     10
    // );
    // For final classified cones
    this->classified_cones_output_rviz_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        this->classified_cones_output_rviz_topic,
        10
    );
    /*
    {
        visualization_msgs::msg::Marker reference_vehicle_marker;
        reference_vehicle_marker.header.frame_id = this->fixed_frame;        // or any TF frame like "map", "odom"
        reference_vehicle_marker.header.stamp = rclcpp::Clock().now();
        reference_vehicle_marker.ns = this->namespace_;
        reference_vehicle_marker.id = 0;
        reference_vehicle_marker.type = visualization_msgs::msg::Marker::CUBE;
        reference_vehicle_marker.action = visualization_msgs::msg::Marker::ADD;
        // Position
        reference_vehicle_marker.pose.position.x = 0.0;
        reference_vehicle_marker.pose.position.y = 0.0;
        reference_vehicle_marker.pose.position.z = 0.25;
        reference_vehicle_marker.pose.orientation.x = 0.0;
        reference_vehicle_marker.pose.orientation.y = 0.0;
        reference_vehicle_marker.pose.orientation.z = 0.0;
        reference_vehicle_marker.pose.orientation.w = 1.0;
        reference_vehicle_marker.scale.x = -(this->rear_end_x);
        reference_vehicle_marker.scale.y = 0.5;
        reference_vehicle_marker.scale.z = 0.5;
        // Color (RGBA)
        reference_vehicle_marker.color.r = 1.0;
        reference_vehicle_marker.color.g = 0.0;
        reference_vehicle_marker.color.b = 0.0;
        reference_vehicle_marker.color.a = 1.0;

        this->reference_vehicle_rviz_pub->publish(reference_vehicle_marker); // Car

        reference_vehicle_marker.id = 1;
        reference_vehicle_marker.type = visualization_msgs::msg::Marker::CYLINDER;
        reference_vehicle_marker.color.r = 0.0;reference_vehicle_marker.color.g = 1.0;
        reference_vehicle_marker.scale.x = 0.1;
        reference_vehicle_marker.scale.y = 0.1;
        reference_vehicle_marker.scale.z = 0.1;
        reference_vehicle_marker.pose.position.x = -(this->rear_end_x)/2-0.025;
        reference_vehicle_marker.pose.position.y = 0.0;
        reference_vehicle_marker.pose.position.z = 0.5+0.05;

        this->reference_vehicle_rviz_pub->publish(reference_vehicle_marker); // Lidar sensor

    }*/

}


void ProcessLidar :: lidar_raw_sub_callback(const sensor_msgs::msg::PointCloud::SharedPtr msg){
// Publishing raw data and initialising position and intensity vectors
    std::vector<std::vector<double>> positions,colors;
    std::vector<double> intensities;
    double max_intensity = *std::max_element(msg->channels[0].values.begin(), msg->channels[0].values.end());
    double min_intensity = *std::min_element(msg->channels[0].values.begin(), msg->channels[0].values.end());

    //RANSAC
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_point_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    for(auto &pt: msg->points){
        pcl_point_cloud->points.push_back(pcl::PointXYZ(pt.x, pt.y, pt.z));
    }

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr ground_pts(new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(this->ransac_threshold);

    seg.setInputCloud(pcl_point_cloud);
    seg.segment(*ground_pts, *coefficients);

        //excluding ground pts
    std::vector<size_t> indices_to_delete;
    for(auto &indice : ground_pts->indices){
        indices_to_delete.push_back(indice);
    }
    eraseIndices(msg, indices_to_delete, positions, intensities);


    

    // for (size_t i = 0; i < msg->points.size(); ++i) {
    //     const auto& pt = msg->points[i];
    //     if(pt.z<this->lidar_z_threshhold){continue;} // Filter out ground use thresholding
    //     // Normalising and adding intensities to the intensities vector
    //     // (i-min_i)/(max_i-min_i)
    //     double norm_intensity = (msg->channels[0].values[i] - min_intensity)/(max_intensity-min_intensity);
    //     colors.push_back({norm_intensity, (0.5-std::abs(0.5-norm_intensity))*2, 1.0-norm_intensity});  // RGB (highest to lowest intensity)
    //     intensities.push_back(msg->channels[0].values[i]);
    //     positions.push_back({pt.x, pt.y, pt.z});
    // }

    // this->publishMarkerArray(
    //     visualization_msgs::msg::Marker::SPHERE,
    //     "marker",
    //     this->fixed_frame,
    //     {positions, colors},
    //     this->lidar_raw_output_rviz_pub,
    //     true,
    //     {0.03, 0.03, 0.03}
    // );

// Performing DBSCAN to identify cone groups
    if (positions.size()==0) {return;}
    // Converting PointCloud data to open3d point cloud
    auto o3d_pcd = std::make_shared<open3d::geometry::PointCloud>();
    for (const auto& point : positions) {
        // Each point is geometry_msgs::msg::Point32 (float x,y,z)
        // Convert to Eigen Vector3d (double)
        o3d_pcd->points_.emplace_back(static_cast<double>(point[0]),
                                        static_cast<double>(point[1]),
                                        static_cast<double>(point[2]));
    }

    std::vector<int> labels = o3d_pcd->ClusterDBSCAN(this->dbscan_epsilon, this->dbscan_minpoints);
    int num_labels = *std::max_element(labels.begin(), labels.end())+1;
// Classifying points
    // This is a vector with k(num of classes) vectors, which are 4x1 vectos of points in that class as {x,y,z,intensity}
    std::vector<std::vector<std::vector<double>>> classified_points(num_labels, std::vector<std::vector<double>>(0, std::vector<double>(4, 0.0)));
    for (size_t index = 0; index < positions.size(); index++) {
        int label = labels[index];
        if (label==-1){continue;} // Filtering out outliers
        auto point = positions.at(index);
        classified_points[label].push_back({point[0], point[1], point[2], intensities[index]});
    }
// Sorting each class's elements in  descending order of z coordinates, top down
    for(auto &cone_class : classified_points){
        std::sort(cone_class.begin(), cone_class.end(), [](const std::vector<double>& v1, const std::vector<double>& v2) -> bool {
            return v1[2] > v2[2]; // sorting by z coordinate, descending
        });
    }
    
// Creating cone xyzs
    colors.clear(); positions.clear();
    for (auto &class_ : classified_points){
        int class_size = class_.size();
        std::vector<double> intensity_vals, z_vals; // For later classification
        if(class_size<10){continue;} // Exclude very far away cones
        double cone_x=0.0, cone_y=0.0, cone_z=0.025;
        for(auto &pt : class_){
            cone_x += pt.at(0); cone_y += pt.at(1);
            intensity_vals.push_back(pt.at(3));
            z_vals.push_back(pt.at(2));
        }
        cone_x = cone_x / class_size; cone_y = cone_y / class_size;

    // // Classifiying type of cone
    //     // Taking a moving average to smoothen out
    //     std::vector<double> averaged_intensities;
    //     int kernel;
    //     if(class_size>150){
    //         kernel = class_size/10;
    //         for(int index = 0; index<class_size-kernel+1; index++){
    //             double grad = 0.0;
    //             for (int count = 0; count < kernel; count++) {
    //                 grad+=class_.at(index+count).at(3);
    //             }
    //             averaged_intensities.push_back(grad/kernel);
    //         }
    //     } else {
    //         kernel = 1;
    //         for(auto point : class_){averaged_intensities.push_back(point.at(3));}
    //     }

    //     // Classification
    //     double gradient=0.0;
    //     double z_mid = (class_[0].at(2) - class_.back().at(2))/2;
    //     for(size_t index = 0; index<averaged_intensities.size()-1; index++){
    //         gradient+=((averaged_intensities.at(index)-averaged_intensities.at(index+1))/(class_.at(index+kernel/2).at(2) - class_.at(index+kernel/2+1).at(2)))*(class_.at(index+kernel/2).at(2)>z_mid?1:-1);
    //     }
    //     gradient=gradient/averaged_intensities.size();
    //     std::cout<<gradient<<std::endl;
    //     if (gradient>0){
    //         colors.push_back({1.0,0.0,0.0});
    //         // std::cout<<"red"<<std::endl;
    //     } else {
    //         colors.push_back({0.0,0.0,1.0});
    //         // std::cout<<"blue"<<std::endl;
    //     }


        // Moving average to smoothen out
        int kernel = std::max(3, static_cast<int>(0.1 * intensity_vals.size()));
        if (kernel % 2 == 0) kernel += 1;

        std::vector<double> averaged_intensity_vals = this->movingAverage(intensity_vals, kernel);

        positions.push_back({cone_x, cone_y, cone_z});
        if (this->classifyCone(averaged_intensity_vals, z_vals)) {
            colors.push_back({1.0, 1.0, 0.0});  // Yellow
        } else {
            colors.push_back({0.0, 0.0, 1.0});  // Blue
        }

        

    }
        
// Publishing classified cones
    this->publishMarkerArray(
        visualization_msgs::msg::Marker::CYLINDER,
        this->namespace_,
        this->fixed_frame,
        {positions, colors},
        this->classified_cones_output_rviz_pub,
        true,
        {0.1, 0.1, 0.5}
    );


}

void ProcessLidar :: publishMarkerArray(visualization_msgs::msg::Marker::_type_type type, std::string ns, std::string frame_id, std::vector<std::vector<std::vector<double>>> positions_colours,rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher, bool del_markers, std::vector<double> scales){
    visualization_msgs::msg::MarkerArray marker_array;
    if(del_markers){
        visualization_msgs::msg::Marker del_marker;
        del_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(del_marker);
    }
    for (size_t i=0; i < positions_colours.at(0).size(); i++){
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = this->now();
        marker.ns = ns;
        marker.id = i;
        marker.type = type;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = positions_colours.at(0).at(i).at(0);
        marker.pose.position.y = positions_colours.at(0).at(i).at(1);
        marker.pose.position.z = positions_colours.at(0).at(i).at(2);
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = scales.at(0);
        marker.scale.y = scales.at(1);
        marker.scale.z = scales.at(2); 

        marker.color.a = 1.0;
        marker.color.r = positions_colours.at(1).at(i).at(0);
        marker.color.g = positions_colours.at(1).at(i).at(1);
        marker.color.b = positions_colours.at(1).at(i).at(2);

        marker_array.markers.push_back(marker);
    }
    publisher->publish(marker_array);

}

bool ProcessLidar :: classifyCone(const std::vector<double> y_vals, std::vector<double> x_vals) {
    int n = y_vals.size();
    Eigen::MatrixXd A(n, 3);
    Eigen::VectorXd y(n);

    // init A and y
    for (int i = 0; i < n; ++i) {
        double x = x_vals.at(i);
        A(i, 0) = x * x;  // t^2
        A(i, 1) = x;      // t
        A(i, 2) = 1.0;    // constant term
        y(i) = y_vals.at(i);
    }

    // Solve for x in least squares sense: A * x = z
    Eigen::Vector3d coeffs = A.colPivHouseholderQr().solve(y);

    // Check curvature
    if (coeffs(0) > 0) {
        return true;
    } else {
        return false;
    }
}

std::vector<double> ProcessLidar :: movingAverage(const std::vector<double>& data, int kernel) {
    int n = data.size();
    std::vector<double> result(n, 0.0);
    if (kernel < 1) return result;
    int half = kernel / 2;
    for (int i = 0; i < n; ++i) {
        int start = std::max(0, i - half);
        int end = std::min(n - 1, i + half);
        double sum = 0.0;
        int count = 0;

        for (int j = start; j <= end; ++j) {
            sum += data[j];
            ++count;
        }

        result[i] = sum / count;
    }
    return result;
}