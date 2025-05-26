/*
*
*
*/
#include "glider/ros/factor_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <map>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    rclcpp::NodeOptions options;

    auto node = std::make_shared<glider::FactorManagerNode>(options);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    
    executor.spin();
    
    rclcpp::shutdown();
    
    return 0;
}
