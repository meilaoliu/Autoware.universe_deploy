#include <memory>
#include <inttypes.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <string>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/time.hpp>
using std::placeholders::_1;

class ImuDataToTf : public rclcpp::Node
{
public:
    ImuDataToTf() : Node("imu_tf_node")
    {
        this->declare_parameter<std::string>("imu_topic", "/sensing/imu/tamagawa/imu_raw");
        this->declare_parameter<std::string>("imu_frame_id", "tamagawa/imu_link");
        this->declare_parameter<int>("position_x", 1);
        this->declare_parameter<int>("position_y", 1);
        this->declare_parameter<int>("position_z", 1);

        this->get_parameter("imu_topic", imu_topic_);
        this->get_parameter("imu_frame_id", imu_frame_id_);
        this->get_parameter("position_x", position_x_);
        this->get_parameter("position_y", position_y_);
        this->get_parameter("position_z", position_z_);

        br_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        sub_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic_, 10, std::bind(&ImuDataToTf::ImuCallback, this, _1));
    }

private:
    void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_data)
    {
        tf2::Quaternion q;
        q.setX(imu_data->orientation.x);
        q.setY(imu_data->orientation.y);
        q.setZ(imu_data->orientation.z);
        q.setW(imu_data->orientation.w);
        q.normalize();

        geometry_msgs::msg::TransformStamped tfs;
        tfs.header.stamp = this->now();
        tfs.header.frame_id = "base_link";
        tfs.child_frame_id = imu_frame_id_;
        tfs.transform.translation.x = position_x_;
        tfs.transform.translation.y = position_y_;
        tfs.transform.translation.z = position_z_;
        tfs.transform.rotation.x = q.getX();
        tfs.transform.rotation.y = q.getY();
        tfs.transform.rotation.z = q.getZ();
        tfs.transform.rotation.w = q.getW();
        br_->sendTransform(tfs);
    }

    std::string imu_topic_;
    std::string imu_frame_id_;
    int position_x_;
    int position_y_;
    int position_z_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> br_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuDataToTf>());
    rclcpp::shutdown();
    return 0;
}
