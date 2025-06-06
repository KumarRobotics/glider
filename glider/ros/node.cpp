/*
*
*
*/

#include "glider/ros/factor_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp/rclcpp.hpp>


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;

    try
    {
        rclcpp::Node = std::make_shared<glider::FactorNode> node(options);
        rclcpp::executors::MultiThreadedExecutor executor;

        executor.add_node(node);

        executor.spin()

        rclcpp::shutdown();
    }
    catch (const std::exception& e)
    {
        std::cerr << "[GLIDER] Caught error " << e.what() << std::endl;
    }
    return 0;
}
