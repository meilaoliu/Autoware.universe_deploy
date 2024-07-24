#include "gnss_transform/gnss_transform.hpp"
// #include "Vector3.h"

namespace gnss_transform {

    using std::placeholders::_1;

    nodeGnssTransform::nodeGnssTransform()
    : rclcpp::Node("gnss_transform_node"),
    tfBroadcaster_(*this)  // 需要再次确定
    {
        // Subscribe to the nav_sat_fix and imu topics
        subGps_ = this->create_subscription<Gpfpd>(
            "GPFPD", 10, std::bind(&gnss_transform::nodeGnssTransform::gpsCallback, this, _1));
        subImu_ = this->create_subscription<Gtimu>(
            "GTIMU", 10, std::bind(&gnss_transform::nodeGnssTransform::imuCallback, this, _1));
        pubTransformedGps_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "odom", 10);
        pubStampedImu_ = this->create_publisher<sensor_msgs::msg::Imu>(
            "imu", 10);
        
        node_rate_ = this->declare_parameter<int>("node_rate", 1);
        base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
        world_frame_ = this->declare_parameter<std::string>("world_frame", "map");
        imu_frame_ = this->declare_parameter<std::string>("imu_frame", "imu");
        // if (!this->get_parameter<int>("node_rate", node_rate_)) {
        //     RCLCPP_WARN(this->get_logger(), "Did not load Parameter:node_rate.");
        // }
        // if (!this->get_parameter<std::string>("base_frame", base_frame_)) {
        //     RCLCPP_WARN(this->get_logger(), "Did not load Parameter:base_frame.");
        // }
        // if (!this->get_parameter<std::string>("world_frame", world_frame_)) {
        //     RCLCPP_WARN(this->get_logger(), "Did not load Parameter:world_frame.");
        // }

        // 后续需要完善得更优雅
        geometry_msgs::msg::Vector3 translationTemp;
        translationTemp.set__x(0);
        translationTemp.set__y(0);
        translationTemp.set__z(0);
        msg_gnss_antenna2base_link_.set__translation(translationTemp);
        msg_gnss_antenna2base_link_.set__rotation(Euler2Quaternion(0, 0 ,0));

        coorconv::WGS84Corr ll;
        coorconv::UTMCoor xyUTM;
        ll.lat = 40.153477;
        ll.lon = 116.264077;
        coorconv::LatLonToUTMXY(ll, xyUTM);
        translationTemp.set__x(xyUTM.x);
        translationTemp.set__y(xyUTM.y);
        translationTemp.set__z(0);
        msg_utm2map_.set__translation(translationTemp);
        msg_utm2map_.set__rotation(Euler2Quaternion(0, 0 ,0));
    }

    nodeGnssTransform::~nodeGnssTransform() {}

    void nodeGnssTransform::gpsCallback(Gpfpd msg)
    {
        RCLCPP_INFO(this->get_logger(), "Receive GPFPD.");
        nav_msgs::msg::Odometry msgTransformedGps;  // 事实上不需要协方差项
        geometry_msgs::msg::TransformStamped transformStamped;
        // Perform transformation on the nav_sat_fix message
        coorconv::WGS84Corr llWGS84Coor;
        coorconv::UTMCoor xyUTMCoor;
        llWGS84Coor.lat = msg.lat;
        llWGS84Coor.lon = msg.lon;
        coorconv::LatLonToUTMXY(llWGS84Coor, xyUTMCoor);

        // Calulate Transform from map to base_link
        // tf2::Vector3 xyzUTM = {xyUTMCoor.x, xyUTMCoor.y, msg.alt};
        tf2::Vector3 xyzUTM = {xyUTMCoor.x, xyUTMCoor.y, 0};
        tf2::Quaternion qTemp;
        qTemp.setRPY(msg.pitch, msg.roll, msg.heading);
        tf2::Quaternion quatTemp;
        quatTemp.setRPY(deg2rad * msg.pitch, deg2rad * msg.roll, deg2rad * msg.heading);
        tf2::convert(quatTemp, vehicle_Quaternion);
        tf2::Transform tf_utm2gnss_antenna{qTemp, xyzUTM};
        tf2::Transform tf_gnss_antenna2base_link;
        tf2::fromMsg(msg_gnss_antenna2base_link_, tf_gnss_antenna2base_link);
        tf2::Transform tf_utm2map;
        tf2::fromMsg(msg_utm2map_, tf_utm2map);
        tf2::Transform tf_map2utm = tf_utm2map.inverse();

        tf2::Transform tf_map2base_link{};
        tf_map2base_link = tf_map2utm *  tf_utm2gnss_antenna * tf_gnss_antenna2base_link;

        msgTransformedGps.header.stamp = this->get_clock()->now();
        msgTransformedGps.header.frame_id = world_frame_;
        msgTransformedGps.child_frame_id = base_frame_;
        
        // set pose
        tf2::toMsg(tf_map2base_link, msgTransformedGps.pose.pose);
        // set twist
        double vx = msg.ve*cos(msg.heading) + msg.vn*sin(msg.heading);
        double vy = msg.ve*cos(msg.heading + M_PI_2) + msg.vn*sin(msg.heading + M_PI_2);
        msgTransformedGps.twist.twist.linear.x = vx;
        msgTransformedGps.twist.twist.linear.y = vy;
        msgTransformedGps.twist.twist.linear.z = msg.vu;

        // Publish the transformed odometry message
        pubTransformedGps_->publish(msgTransformedGps);
        RCLCPP_INFO(this->get_logger(), "Publish Transformed GPS.");

        // Publish the TF
        transformStamped.header.frame_id = world_frame_;
        transformStamped.child_frame_id = base_frame_;
        transformStamped.header.stamp = msgTransformedGps.header.stamp;

        transformStamped.transform.translation.x = msgTransformedGps.pose.pose.position.x;
        transformStamped.transform.translation.y = msgTransformedGps.pose.pose.position.y;
        transformStamped.transform.translation.z = msgTransformedGps.pose.pose.position.z;

        tf2::Quaternion tf_quaternion; // 为什么要新建中间变量？有啥区别？
        tf2::fromMsg(msgTransformedGps.pose.pose.orientation, tf_quaternion);
        transformStamped.transform.rotation.x = tf_quaternion.x();
        transformStamped.transform.rotation.y = tf_quaternion.y();
        transformStamped.transform.rotation.z = tf_quaternion.z();
        transformStamped.transform.rotation.w = tf_quaternion.w();
        // transformStamped.transform.rotation.x = msgTransformedGps.pose.pose.orientation.x;
        // transformStamped.transform.rotation.y = msgTransformedGps.pose.pose.orientation.y;
        // transformStamped.transform.rotation.z = msgTransformedGps.pose.pose.orientation.z;
        // transformStamped.transform.rotation.w = msgTransformedGps.pose.pose.orientation.w;
        tfBroadcaster_.sendTransform(transformStamped);
        RCLCPP_INFO(this->get_logger(), "Publish TF.");
    }

    void nodeGnssTransform::imuCallback(Gtimu msg)
    {
        RCLCPP_INFO(this->get_logger(), "Receive GTIMU.");
        geometry_msgs::msg::AccelStamped msgStampedImu;
        sensor_msgs::msg::Imu msg_IMU_;
        // Perform transformation on the imu message

        msg_IMU_.header.stamp = msg.header.stamp;//todo
        msg_IMU_.header.frame_id = imu_frame_;
        msg_IMU_.angular_velocity.x = msg.ax;
        msg_IMU_.angular_velocity.y = msg.ay;
        msg_IMU_.angular_velocity.z = msg.az;

        msg_IMU_.linear_acceleration.x = deg2rad * msg.gx;
        msg_IMU_.linear_acceleration.y = deg2rad * msg.gy;
        msg_IMU_.linear_acceleration.z = deg2rad * msg.gz;

        msg_IMU_.orientation = vehicle_Quaternion;
        // Publish the transformed imu message
        pubStampedImu_->publish(msg_IMU_);
        RCLCPP_INFO(this->get_logger(), "Publish Stamped IMU.");
    }

    geometry_msgs::msg::Quaternion nodeGnssTransform::Euler2Quaternion(const double& roll, const double& pitch, const double& yaw)
	{
		double halfYaw = double(yaw) * double(0.5);  
		double halfPitch = double(pitch) * double(0.5);  
		double halfRoll = double(roll) * double(0.5);  
		double cosYaw = cos(halfYaw);
		double sinYaw = sin(halfYaw);
		double cosPitch = cos(halfPitch);
		double sinPitch = sin(halfPitch);
		double cosRoll = cos(halfRoll);
		double sinRoll = sin(halfRoll);

        geometry_msgs::msg::Quaternion result;
        result.set__x(sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw);
        result.set__y(cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw);
        result.set__z(cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw);
        result.set__w(cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw);
        return result;
	}
} // namespace gnss_transform