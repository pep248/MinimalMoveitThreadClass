#ifndef IK_UR_ROUTINE_CLASS_HPP_
#define IK_UR_ROUTINE_CLASS_HPP_

#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>

#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <graph_msgs/msg/geometry_graph.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "std_srvs/srv/empty.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <vector>

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

    moveit::planning_interface::MoveGroupInterface::PlanPtr moveit_plan_;

    void executeMotion();
	void planAndExecuteTrajectory(
        const std::vector<geometry_msgs::msg::TransformStamped>& waypoints);
    void planAndExecuteModifiedTrajectory();

    void loadPosesIntoWaypointsVector(
    std::vector<geometry_msgs::msg::Pose>* waypoints_,
    std::string file_path,
    std::shared_ptr<std::vector<geometry_msgs::msg::TransformStamped>> transforms_vector_,
    std::shared_ptr<std::vector<geometry_msgs::msg::Pose>> poses_vector_);

	void waitForUserInput();
    void addCollisionChair();
    void importTransformsFromYAML(
        const std::string& yamlFilePath, 
        std::shared_ptr<std::vector<geometry_msgs::msg::TransformStamped>> transforms_vector_,
        std::shared_ptr<std::vector<geometry_msgs::msg::Pose>> poses_vector_
    );

    std::shared_ptr<rclcpp::CallbackGroup> subscription_callback_group_;
    rclcpp::SubscriptionOptions subscription_options_;

    // std::shared_ptr<rclcpp::CallbackGroup> service_callback_group_;
    // rclcpp::ServiceOptions service_options_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr chair_subscription_;
    void chair_pose_callback(geometry_msgs::msg::PoseStamped::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr rviz_subscription_;
    void rviz_callback(sensor_msgs::msg::Joy::SharedPtr msg);
    int task_state;
        // 0 = stop
        // 1 = running
        // 2 = waiting
    int wait_for_rviz();
    
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr marker_subscription_;
    void markerCallback(visualization_msgs::msg::Marker::SharedPtr msg);

    void transformVectors(
        std::shared_ptr<std::vector<geometry_msgs::msg::Pose>> poses_vector_,
        std::shared_ptr<std::vector<geometry_msgs::msg::TransformStamped>> transforms_vector_);

    geometry_msgs::msg::TransformStamped::SharedPtr chair_transform_;
    int chair_state;
        // 1 = right
        // 2 = left
        // 3 = front
        // 4 = back
        // 5 = error/unknown
        
    std::string right_file_path;
    std::shared_ptr<std::vector<geometry_msgs::msg::TransformStamped>> right_transforms_vector_;
    std::shared_ptr<std::vector<geometry_msgs::msg::Pose>> right_poses_vector_;
    std::string left_file_path;
    std::shared_ptr<std::vector<geometry_msgs::msg::TransformStamped>> left_transforms_vector_;
    std::shared_ptr<std::vector<geometry_msgs::msg::Pose>> left_poses_vector_;
    std::string front_file_path;
    std::shared_ptr<std::vector<geometry_msgs::msg::TransformStamped>> front_transforms_vector_;
    std::shared_ptr<std::vector<geometry_msgs::msg::Pose>> front_poses_vector_;
    std::string back_file_path;
    std::shared_ptr<std::vector<geometry_msgs::msg::TransformStamped>> back_transforms_vector_;
    std::shared_ptr<std::vector<geometry_msgs::msg::Pose>> back_poses_vector_;
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_pub_;

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr empty_service_;
    void handleEmptyService(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                            std::shared_ptr<std_srvs::srv::Empty::Response> response);
};

#endif  // IK_UR_ROUTINE_CLASS_HPP_
