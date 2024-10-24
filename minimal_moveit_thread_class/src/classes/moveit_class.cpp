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
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;
    moveit_visual_tools_.publishText(text_pose, "CALLBACK", rvt::WHITE, rvt::XLARGE);
	moveit_visual_tools_.trigger();
    // This function will be called w
    henever the service is requested
    // Since it's an empty service, we don't need to do anything here
    // RCLCPP_INFO(this->get_logger(), "Empty service called.");
    RCLCPP_WARN(this->get_logger(), "H");
    while(1)
    {
	    this->wait_for_rviz();
        this->planAndExecuteModifiedTrajectory();
    }
    
    RCLCPP_WARN(this->get_logger(), "J");
}


void MoveitClass::planAndExecuteModifiedTrajectory()
{
    std::vector<geometry_msgs::msg::Pose> waypoints;

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.001;
    RCLCPP_WARN(this->get_logger(), "K");
    // double fraction = this->move_group_interface_.computeCartesianPath(*this->right_poses_vector_, eef_step, jump_threshold, trajectory);
    this->move_group_interface_.setMaxAccelerationScalingFactor(0.1);
    double fraction = this->move_group_interface_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    RCLCPP_WARN(this->get_logger(), "L");
    if (fraction < 1)
    {
        RCLCPP_ERROR(this->get_logger(), "Could not compute trajectory through all waypoints!");
    }
    else
    {
        this->move_group_interface_.execute(trajectory) ==  moveit::core::MoveItErrorCode::SUCCESS;
    }

    // // Plan and execute trajectory
    // // move_group_interface_->
    // move_group_interface_->setPoseTarget(pose);
    // this->executeMotion();
}


void MoveitClass::loadPosesIntoWaypointsVector(
    std::vector<geometry_msgs::msg::Pose>* waypoints_,
    std::string file_path,
    std::shared_ptr<std::vector<geometry_msgs::msg::TransformStamped>> transforms_vector_,
    std::shared_ptr<std::vector<geometry_msgs::msg::Pose>> poses_vector_)
{
    if (!file_path.empty())
    {
        this->importTransformsFromYAML(file_path, transforms_vector_, poses_vector_);
        RCLCPP_WARN(this->get_logger(), "Length of the right vector: %li", transforms_vector_->size());
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "No path provided for %s. No transforms will be imported.",file_path.c_str());
    }

    
    RCLCPP_WARN(this->get_logger(), "Length of poses_vector_: %lu", poses_vector_->size());

    if(poses_vector_->size() != 0)
    {   
        geometry_msgs::msg::TransformStamped current_transform = this->tf_buffer_.lookupTransform("world", "tool0", rclcpp::Time(0));
        geometry_msgs::msg::Pose current_pose;
        current_pose.position.x = current_transform.transform.translation.x;
        current_pose.position.y = current_transform.transform.translation.y;
        current_pose.position.z = current_transform.transform.translation.z;
        current_pose.orientation.x = current_transform.transform.rotation.x;
        current_pose.orientation.y = current_transform.transform.rotation.y;
        current_pose.orientation.z = current_transform.transform.rotation.z;
        current_pose.orientation.w = current_transform.transform.rotation.w;
        waypoints_->push_back(current_pose);
        // forward motion
        for (const auto& pose : *poses_vector_) {
            waypoints_->push_back(pose);
        }
    }
}


int MoveitClass::wait_for_rviz()
{   
    RCLCPP_WARN(this->get_logger(), "Waiting");
    this->task_state = 2; // Waiting
    while(this->task_state == 2) // Running
    {
        rclcpp::sleep_for(std::chrono::seconds(1));
    }
    if (this->task_state == 1) // Running
    {
        RCLCPP_WARN(this->get_logger(), "Running");
        return 1;
    }
    else // Stop
    {
        RCLCPP_WARN(this->get_logger(), "Stopping");
        return 0;
    }
}


// CALLBACKS

void MoveitClass::rviz_callback(sensor_msgs::msg::Joy::SharedPtr msg)
{
    RCLCPP_WARN(this->get_logger(), "RVIZ");
    if (msg->buttons[1] == 1) // Next
    {
        RCLCPP_WARN(this->get_logger(), "Next");
        if (this->task_state == 2) // Waiting
        {
            this->task_state = 1; // Running
        }
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "NOT Next");
        this->task_state = 0; // Stop
    }
}