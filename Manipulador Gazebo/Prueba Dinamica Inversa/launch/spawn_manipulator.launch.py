# launch/six_dof_manipulator.launch.py
import os
import xacro

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('six_dof_manipulator_description')
    xacro_file = os.path.join(pkg_share, 'urdf', '6dof_manipulator.xacro')
    doc = xacro.process_file(xacro_file)
    robot_description_xml = doc.toxml()

    # 2) Build the robot_state_publisher parameter dict
    robot_description = {'robot_description': robot_description_xml}

    # — Gazebo server (classic) —
    gzserver = ExecuteProcess(
        cmd=[
          'gzserver', '--verbose',
          '-s', 'libgazebo_ros_init.so',
          '-s', 'libgazebo_ros_factory.so'
        ],
        output='screen'
    )

    # — Gazebo client —
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    # — spawn our URDF into Gazebo after it’s up —
    spawn = TimerAction(
        period=2.0,  # give Gazebo time to start
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
    )

    return LaunchDescription([
        gzserver,
        gzclient,

        # 3) Publish TFs from the URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description],
            output='screen',
        ),

        spawn,

        # 4) Your Python dynamics publisher
        Node(
            package='six_dof_manipulator_description',
            executable='dynamics_publisher.py',
            name='dynamics_publisher',
            output='screen',
        ),

        # 5) Your two C++ controller nodes
        Node(
            package='six_dof_manipulator_description',
            executable='inverse_dynamics_controller_node',
            name='inverse_dynamics_controller',
            output='screen',
        ),
        #Node(
        #    package='six_dof_manipulator_description',
        #    executable='torque_applier_node',
        #    name='torque_applier',
        #    output='screen',
        #),
    ])
