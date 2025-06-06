/**
* @name process_lidar.hpp
* @brief Header (Declaration / Interface) file for the node
* @author Siddhesh Phadke
*/

#ifndef PROCESS_LIDAR_HPP
#define PROCESS_LIDAR_HPP
#include "rclcpp/rclcpp.hpp" // Node class inheritance
#include "sensor_msgs/msg/point_cloud.hpp" // Reading point cloud messsages



/**
* @brief Node class Interface
*/
class ProcessLidar : public rclcpp::Node {
private:
    
public:
    // Constructor
    ProcessLidar();
};




#endif