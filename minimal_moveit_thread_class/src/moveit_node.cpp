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
    auto moveit_thread_node = std::make_shared<MoveitClass>();

    // Start the spinner thread as soon as possible
    std::thread spin_thread(executor_spin, moveit_thread_node);

    // Start the routine
    moveit_thread_node->StartRoutine();

    rclcpp::shutdown();
    return 0;
}