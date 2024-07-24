#include <rclcpp/rclcpp.hpp>

#include "gnss_transform/gnss_transform.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<gnss_transform::nodeGnssTransform>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
