import os
import sys

import launch
import launch_ros.actions

def generate_launch_description():

    rviz_config_dir_global = os.path.join(os.getcwd(), 'install/recorder/share/recorder', 'my_config_global.rviz')
    rviz_config_dir_follow = os.path.join(os.getcwd(), 'install/recorder/share/recorder', 'my_config_follow.rviz')

    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='recorder',
            executable='TrajectoryRecorder',
            name=['TrajectoryRecorder_'],
            output='screen',
            emulate_tty=True,
            parameters=[{}]
        ), 
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name=['rviz_global'],
            output='screen',
            arguments=['-d', rviz_config_dir_global]
        ),
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name=['rviz_follow'],
            output='screen',
            arguments=['-d', rviz_config_dir_follow]
        )
    ])
    return ld

if __name__ == '__main__':
    generate_launch_description()