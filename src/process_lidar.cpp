/**
* @file process_lidar.cpp
* @brief Source (Definition / Implementation) file for the node
* @author Siddhesh Phadke
*/

#include "perception_winter/process_lidar.hpp"

ProcessLidar :: ProcessLidar() : Node("process_lidar"){
    RCLCPP_INFO(this->get_logger(), "test");
}
