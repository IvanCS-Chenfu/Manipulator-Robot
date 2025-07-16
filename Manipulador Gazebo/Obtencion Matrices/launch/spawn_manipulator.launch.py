import os
import xacro

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1) Locate & expand your Xacro to XML text
    pkg_share = get_package_share_directory('six_dof_manipulator_description')
    xacro_file = os.path.join(pkg_share, 'urdf', '6dof_manipulator.xacro')
    doc = xacro.process_file(xacro_file)
    robot_description_xml = doc.toxml()

    # 2) Build the robot_state_publisher parameter dict
    robot_description = {'robot_description': robot_description_xml}

    return LaunchDescription([
        # ——————— Gazebo server ———————
        ExecuteProcess(
            cmd=[
                'gzserver', '--verbose',
                '-s', 'libgazebo_ros_init.so',
                '-s', 'libgazebo_ros_factory.so'
            ],
            output='screen'
        ),

        # ——————— Gazebo client ———————
        ExecuteProcess(
            cmd=['gzclient'],
            output='screen'
        ),

        # ——— Publish robot_description ———
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[ robot_description ],
        ),

        # ——— Spawn the entity after a short delay ———
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=[
                        '-entity', 'six_dof_manipulator',
                        '-topic', 'robot_description',
                        '-timeout', '30'
                    ],
                    output='screen'
                )
            ]
        ),
    ])
