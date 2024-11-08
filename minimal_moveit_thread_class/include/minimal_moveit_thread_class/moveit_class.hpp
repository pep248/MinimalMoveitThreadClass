#ifndef MOVEIT_CLASS_HPP_
#define MOVEIT_CLASS_HPP_

// Core includes
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

class MoveitClass : public rclcpp::Node
{
public:
    MoveitClass();
    void StartRoutine();
    
private:
    const std::string MOVE_GROUP_NAME = "ur_manipulator";
    const std::string BASE_LINK = "base_link";
    moveit::planning_interface::MoveGroupInterface move_group_interface_;
    moveit_visual_tools::MoveItVisualTools moveit_visual_tools_;
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

#endif  // MOVEIT_CLASS_HPP_