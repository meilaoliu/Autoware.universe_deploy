#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_vehicle_msgs.msg import ControlModeReport, GearReport, SteeringReport, VelocityReport
from hunter_msgs.msg import HunterStatus
from tier4_vehicle_msgs.msg import BatteryStatus

class VehicleInterface(Node):
    def __init__(self):
        super().__init__('vehicle_interface')
        
        # 订阅 Autoware 的控制指令
        self.subscriber_control_cmd = self.create_subscription(
            AckermannControlCommand,
            'control/command/control_cmd',
            self.control_cmd_callback,
            10
        )
        
        # 发布小车的控制指令
        self.publisher_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 订阅小车的状态信息
        self.subscriber_hunter_status = self.create_subscription(
            HunterStatus,
            '/hunter_status',
            self.hunter_status_callback,
            10
        )
        
   
        # 发布转向状态
        self.publisher_steering = self.create_publisher(SteeringReport, '/vehicle/status/steering_status', 10)
        
        # 发布速度状态
        self.publisher_velocity = self.create_publisher(VelocityReport, '/vehicle/status/velocity_status', 10)
        
        # 发布电池状态
        self.publisher_battery = self.create_publisher(BatteryStatus, '/vehicle/status/battery_charge', 10)

    def control_cmd_callback(self, msg):
        """
        处理从 Autoware 收到的控制命令，并将其转换为 Ackermann 结构小车的控制命令
        """
        twist = Twist()
        twist.linear.x = msg.longitudinal.speed  # 线速度
        twist.angular.z = msg.lateral.steering_tire_angle  # 转向角度
        
        self.publisher_cmd_vel.publish(twist)  # 发布转换后的控制命令

    def hunter_status_callback(self, msg):
        """
        处理从小车接收到的状态信息，并将其发布到 Autoware 的相关话题
        """
   
        # 发布转向报告
        steering_report = SteeringReport()
        steering_report.stamp = self.get_clock().now().to_msg()
        steering_report.steering_tire_angle = msg.steering_angle
        self.publisher_steering.publish(steering_report)

        # 发布速度报告
        velocity_report = VelocityReport()
        velocity_report.header.stamp = self.get_clock().now().to_msg()
        velocity_report.header.frame_id = "base_link"
        velocity_report.longitudinal_velocity = msg.linear_velocity  # 纵向速度
        velocity_report.lateral_velocity = 0.0  # 横向速度（假设为0）
        velocity_report.heading_rate = msg.steering_angle  # 航向变化率

        self.publisher_velocity.publish(velocity_report)

        # 发布电池状态报告
        battery_status = BatteryStatus()
        battery_status.stamp = self.get_clock().now().to_msg()  # 时间戳
        battery_status.energy_level = msg.battery_voltage  # 电池电压
        self.publisher_battery.publish(battery_status)

def main(args=None):
    rclpy.init(args=args)
    node = VehicleInterface()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
