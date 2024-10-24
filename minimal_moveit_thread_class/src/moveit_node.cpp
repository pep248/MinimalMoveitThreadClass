#include <minimal_moveit_thread_class/moveit_class.hpp>

using namespace std;

void executor_spin(std::shared_ptr<rclcpp::Node> node)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // Create an instance of the MinimalActionClient()
    auto ik_routine_node = std::make_shared<MoveitClass>();
    // Initialize the moveit pointers
    // ik_routine_node->init();
    

    // Start the spinner thread as soon as possible
    std::thread spin_thread(executor_spin, ik_routine_node);

    ik_routine_node->detachedRoutine();

    // // Create an executor and put the action_client inside
    // auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    // executor->add_node(ik_routine_node);

    // // Check if the action was successful and kill the node once finished
    // std::thread([executor]() { executor->spin(); }).detach();

    // // ik_routine_node->moveit_visual_tools_->prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    rclcpp::shutdown();
    return 0;
}