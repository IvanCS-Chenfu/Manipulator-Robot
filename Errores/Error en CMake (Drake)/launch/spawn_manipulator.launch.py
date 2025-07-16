import os

from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('six_dof_manipulator_description')
    xacro_file = os.path.join(pkg_dir, 'urdf', '6dof_manipulator.xacro')

    # Process the xacro file to URDF via the Command substitution
    robot_description_content = Command(['xacro ', xacro_file])
    robot_description = {'robot_description': robot_description_content}

    return LaunchDescription([
                # Launch Gazebo server with ROS2 plugins
        ExecuteProcess(
            cmd=[
                'gzserver', '--verbose',
                '-s', 'libgazebo_ros_init.so',
                '-s', 'libgazebo_ros_factory.so'
            ],
            output='screen'
        ),
        # Launch Gazebo client
        ExecuteProcess(
            cmd=['gzclient'], output='screen'
        ),
        # Publish robot_description via robot_state_publisher
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            output='screen', parameters=[robot_description]
        ),
        # Delay spawn until robot_state_publisher is ready
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='gazebo_ros', executable='spawn_entity.py',
                    arguments=[
                        '-entity', 'six_dof_manipulator',
                        '-topic', 'robot_description',
                        '-timeout', '30'
                    ],
                    output='screen'
                )
            ]
        )
    ])