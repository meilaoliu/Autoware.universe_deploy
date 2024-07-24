#ifndef GNSS_TRANSFORM_HPP
#define GNSS_TRANSFORM_HPP

#define ROS_DISTRO_GALACTIC // ros-galactic

#include <string>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/header.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/accel_stamped.hpp"

#include "starneto_ros_msgs/msg/gpfpd.hpp"
#include "starneto_ros_msgs/msg/gtimu.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#ifdef ROS_DISTRO_GALACTIC
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include "gnss_transform/coorconv.hpp"
#define PI 3.1415926

namespace gnss_transform {

    using starneto_ros_msgs::msg::Gpfpd;
    using starneto_ros_msgs::msg::Gtimu;

    const double deg2rad = PI/180;
    class nodeGnssTransform : public rclcpp::Node {
    public:
        nodeGnssTransform();
        ~nodeGnssTransform();

    private:
        void gpsCallback(const Gpfpd msg);
        void imuCallback(const Gtimu msg);
        geometry_msgs::msg::Quaternion Euler2Quaternion(const double& roll, const double& pitch, const double& yaw);  // radian

        const double lonMapOrigin_ = 0.0;
        const double latMapOrigin_ = 0.0;

        int node_rate_;  // 无实际作用
        std::string base_frame_;
        std::string world_frame_;
        std::string imu_frame_;

        rclcpp::Subscription<Gpfpd>::SharedPtr subGps_;
        rclcpp::Subscription<Gtimu>::SharedPtr subImu_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubTransformedGps_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pubStampedImu_;
        
        geometry_msgs::msg::Transform msg_gnss_antenna2base_link_;
        geometry_msgs::msg::Transform msg_utm2map_;
        tf2_ros::TransformBroadcaster tfBroadcaster_; // tf broadcaster map->base_link

        geometry_msgs::msg::Quaternion vehicle_Quaternion;
    };
}
#endif // GNSS_TRANSFORM_H_