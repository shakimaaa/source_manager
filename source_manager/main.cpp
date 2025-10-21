#include "rclcpp/rclcpp.hpp"
#include "source_manager/source_manager.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SourceManager>();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    
    node->init();
    executor.spin();
    rclcpp::shutdown();
    return 0;
}