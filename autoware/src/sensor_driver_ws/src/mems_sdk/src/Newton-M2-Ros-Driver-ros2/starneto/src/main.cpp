/*
  Copyright(c) 2021:
  Huang Chenrui <hcr2077@outlook.com>

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

  http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
*/

#include "rclcpp/rclcpp.hpp"
// #include <slcurses.h>
#include "starneto_mems.hpp"


typedef ns_starneto_mems::Starneto Starneto;

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto nodeHandle = rclcpp::Node::make_shared("starneto_mems");
    // std::shared_ptrrclcpp::Node nodeHandle;
    Starneto myStarneto(nodeHandle);
    myStarneto.initSerial();
    rclcpp::WallRate loop_rate(myStarneto.getNodeRate());

    while (rclcpp::ok()) {
        myStarneto.run();
        myStarneto.sendMsg();
        rclcpp::spin_some(nodeHandle);  // Keeps node alive basically
        loop_rate.sleep(); // Sleep for loop_rate
    }
    rclcpp::shutdown();
    
    // ros::init(argc, argv, "starneto_mems");
    
    // ros::NodeHandle nodeHandle("~");
    // Starneto myStarneto(nodeHandle);  // init object
    // myStarneto.initSerial();
    // ros::Rate loop_rate(myStarneto.getNodeRate());

    // while (ros::ok()) {
    //     myStarneto.run();
    //     myStarneto.sendMsg();
    //     ros::spinOnce();   // Keeps node alive basically
    //     loop_rate.sleep(); // Sleep for loop_rate
    // }
    return 0;
}

Starneto::Starneto(rclcpp::Node::SharedPtr &nodeHandle) : nodeHandle(nodeHandle) {
    loadParameters();
    publishToTopics();
};

int Starneto::getNodeRate() const { return node_rate; }

void Starneto::loadParameters() {
    RCLCPP_INFO(nodeHandle->get_logger(), "loading handle parameters");
    // ROS_INFO("loading handle parameters");
    nodeHandle->declare_parameter<int>("node_rate", 1);
    nodeHandle->declare_parameter<int>("serial_baud", 115200);
    nodeHandle->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    nodeHandle->declare_parameter<std::string>("GPFPD_output_topic", "/GPFPD");
    nodeHandle->declare_parameter<std::string>("GTIMU_output_topic", "/GTIMU");

    if (!nodeHandle->get_parameter<int>("node_rate", node_rate)) {
        RCLCPP_WARN(nodeHandle->get_logger(), "Did not load node_rate.");
    }
    if (!nodeHandle->get_parameter<int>("serial_baud", serial_baud)) {
        RCLCPP_WARN(nodeHandle->get_logger(), "Did not load serial_baud.");
    }
    if (!nodeHandle->get_parameter<std::string>("serial_port", serial_port)) {
        RCLCPP_WARN(nodeHandle->get_logger(), "Did not load serial_port.");
    }
    if (!nodeHandle->get_parameter<std::string>("GPFPD_output_topic", GPFPD_output_topic)) {
        RCLCPP_WARN(nodeHandle->get_logger(), "Did not load GPFPD_output_topic.");
    }
    if (!nodeHandle->get_parameter<std::string>("GTIMU_output_topic", GTIMU_output_topic)) {
        RCLCPP_WARN(nodeHandle->get_logger(), "Did not load GTIMU_output_topic.");
    }
    // if (!nodeHandle.param<int>("node_rate", node_rate, 1)) {
    //     ROS_WARN_STREAM("Did not load node_rate.");
    // }
    // if (!nodeHandle.param<std::string>("serial_port", serial_port, "/dev/ttyUSB0")) {
    //     ROS_WARN_STREAM("Did not load serial_port.");
    // }
    // if (!nodeHandle.param<std::string>("GPFPD_output_topic", GPFPD_output_topic, "/GPFPD")) {
    //     ROS_WARN_STREAM("Did not load GPFPD_output_topic.");
    // }
    // if (!nodeHandle.param<std::string>("GTIMU_output_topic", GTIMU_output_topic, "/GTIMU")) {
    //     ROS_WARN_STREAM("Did not load GTIMU_output_topic.");
    // }
}

void Starneto::publishToTopics() {
    RCLCPP_INFO(nodeHandle->get_logger(), "publish to topics");
    pub_gpfpd = nodeHandle->create_publisher<starneto_ros_msgs::msg::Gpfpd>(GPFPD_output_topic, 10);
    pub_gtimu = nodeHandle->create_publisher<starneto_ros_msgs::msg::Gtimu>(GTIMU_output_topic, 10);
    // ROS_INFO("publish to topics");
    // pub_gpfpd = nodeHandle.advertise<starneto_ros_msgs::Gpfpd>(GPFPD_output_topic, 100);
    // pub_gtimu = nodeHandle.advertise<starneto_ros_msgs::Gtimu>(GTIMU_output_topic, 100);
    // Publisher = nodeHandle.advertise<msg_type>(topic_name_, 1);

}

void Starneto::sendMsg() {
    gnss.header.frame_id = "/gps";
    gnss.header.stamp = nodeHandle->get_clock()->now();
    // gnss.header.stamp = ros::Time::now();//ros时刻
    imu.header.frame_id = "/imu";
    imu.header.stamp = gnss.header.stamp;//gnss time
    pub_gpfpd->publish(gnss);
    pub_gtimu->publish(imu);
    //Publisher.publish(msg);
}

void Starneto::initSerial() {
    try {
        //设置串口属性，并打开串口
        ser.setPort(serial_port);
        ser.setBaudrate(serial_baud);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000); //超时定义，单位：ms
        ser.setTimeout(to);
        ser.open();
        ser.flushInput(); // first clear the buffer
    }
    catch (serial::IOException &e) {
        RCLCPP_ERROR_STREAM(nodeHandle->get_logger(), "serial port: " << serial_port << "init failed.");
        // ROS_ERROR_STREAM("serial port: " << serial_port << "init failed.");
    }
}
