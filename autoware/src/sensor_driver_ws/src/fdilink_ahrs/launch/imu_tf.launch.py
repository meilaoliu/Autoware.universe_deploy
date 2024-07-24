
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fdilink_ahrs',  # 替换为你的包名
            executable='imu_tf_node',  # 替换为你的可执行文件名
            name='imu_data_to_tf_node',
            output='screen',
            parameters=[
                {'imu_topic': '/sensing/imu/tamagawa/imu_raw'},
                {'imu_frame_id': 'tamagawa/imu_link'},
                {'position_x': 1},
                {'position_y': 1},
                {'position_z': 1},
            ]
        )
    ])
