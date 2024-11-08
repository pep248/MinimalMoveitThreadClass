#include <minimal_moveit_thread_class/moveit_class.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

#include "iostream"

using namespace std;
namespace rvt = rviz_visual_tools;

MoveitClass::MoveitClass() : Node("moveit_thread_node"),
    move_group_interface_(
        std::shared_ptr<rclcpp::Node>(
            std::move(this)),
            MOVE_GROUP_NAME),
    moveit_visual_tools_(
        std::shared_ptr<rclcpp::Node>(
            std::move(this)), 
        BASE_LINK, 
        rviz_visual_tools::RVIZ_MARKER_TOPIC,
        move_group_interface_.getRobotModel()),
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_)
{
    this->moveit_visual_tools_.deleteAllMarkers();
    this->moveit_visual_tools_.loadRemoteControl();

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;
    moveit_visual_tools_.publishText(text_pose, "MoveGroupInterface_Demo", rvt::WHITE, rvt::XLARGE);
    moveit_visual_tools_.trigger();
}

void MoveitClass::StartRoutine()
{
    moveit_visual_tools_.prompt("Press 'Next' in the RvizVisualToolsGui to go to the initial position");

    // Initial pose
    geometry_msgs::msg::Pose pose0;
    pose0.position.x = 0.8;
    pose0.position.y = 0.0;
    pose0.position.z = 0.8;
    pose0.orientation.w = 1.0;

    move_group_interface_.setPoseTarget(pose0);
    move_group_interface_.move();

    // Define waypoints for the first trajectory
    moveit_visual_tools_.prompt("Press 'Next' in the RvizVisualToolsGui to start the cartesian trajectory");
    std::vector<geometry_msgs::msg::Pose> waypoints;

    geometry_msgs::msg::Pose pose1 = pose0;
    pose1.position.y = 0.1;

    geometry_msgs::msg::Pose pose2 = pose0;
    pose2.position.y = -0.1;

    geometry_msgs::msg::Pose pose3 = pose0;
    pose3.position.z = 0.6;

    waypoints.push_back(pose1);
    waypoints.push_back(pose2);
    waypoints.push_back(pose3);

    // Plan and execute trajectory
    const double jump_threshold = 0.0;
    const double eef_step = 0.001;
    moveit_msgs::msg::RobotTrajectory trajectory;
    this->move_group_interface_.setMaxAccelerationScalingFactor(0.1);
    double fraction = this->move_group_interface_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    if (fraction < 1.0) {
        RCLCPP_ERROR(this->get_logger(), "Could not compute trajectory through all waypoints!");
    } else {
        RCLCPP_INFO(this->get_logger(), "Executing trajectory...");
        this->move_group_interface_.execute(trajectory);
    }

    // Define waypoints for the second trajectory
    moveit_visual_tools_.prompt("Press 'Next' in the RvizVisualToolsGui to start the second cartesian trajectory");

    waypoints.clear();
    geometry_msgs::msg::Pose pose4;
    pose4.position.x = 1.0;
    pose4.position.y = 0.4;
    pose4.position.z = 1.0;
    pose4.orientation.w = 1.0;

    geometry_msgs::msg::Pose pose5;
    pose5.position.x = 0.6;
    pose5.position.y = 0.4;
    pose5.position.z = 0.6;
    pose5.orientation.w = 1.0;

    geometry_msgs::msg::Pose pose6 = pose0;

    waypoints.push_back(pose4);
    waypoints.push_back(pose5);
    waypoints.push_back(pose6);

    fraction = this->move_group_interface_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    if (fraction < 1.0) {
        RCLCPP_ERROR(this->get_logger(), "Could not compute second trajectory through all waypoints!");
    } else {
        RCLCPP_INFO(this->get_logger(), "Executing second trajectory...");
        this->move_group_interface_.execute(trajectory);
    }

    RCLCPP_INFO(this->get_logger(), "Routine completed.");
}