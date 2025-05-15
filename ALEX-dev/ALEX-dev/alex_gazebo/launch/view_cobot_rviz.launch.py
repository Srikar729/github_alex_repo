# 
#  dsr_bringup2
#  Author: Minsoo Song (minsoo.song@doosan.com)
#  
#  Copyright (c) 2024 Doosan Robotics
#  Use of this source code is governed by the BSD, see LICENSE
# 

import os

from launch import LaunchDescription
from launch.actions import RegisterEventHandler,DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import xacro


ARGUMENTS =[ 
    DeclareLaunchArgument('name',  default_value = '',     description = 'NAME_SPACE'     ),
    DeclareLaunchArgument('model', default_value = 'm1013',     description = 'ROBOT_MODEL'    ),
    DeclareLaunchArgument('color', default_value = 'white',     description = 'ROBOT_COLOR'    ),
    DeclareLaunchArgument('gui',   default_value = 'true',     description = 'Start RViz2'    ),
    DeclareLaunchArgument('use_gazebo',   default_value = 'true',     description = 'Start Gazebo'    ),

    DeclareLaunchArgument('x',   default_value = '0',     description = 'Location x on Gazebo '    ),
    DeclareLaunchArgument('y',   default_value = '0',     description = 'Location y on Gazebo'    ),
    DeclareLaunchArgument('z',   default_value = '0',     description = 'Location z on Gazebo'    ),
    DeclareLaunchArgument('R',   default_value = '0',     description = 'Location Roll on Gazebo'    ),
    DeclareLaunchArgument('P',   default_value = '0',     description = 'Location Pitch on Gazebo'    ),
    DeclareLaunchArgument('Y',   default_value = '0',     description = 'Location Yaw on Gazebo'    ),
]

def generate_launch_description():
    # Initialize Arguments
    gui = LaunchConfiguration("gui")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("alex_gazebo"),
                    "xacro",
                    "a0509",
                ]
            ),
            ".urdf.xacro",
            " ",
            "use_gazebo:=",
            LaunchConfiguration('use_gazebo'),
            " ",
            "color:=",
            LaunchConfiguration('color'),
            " ",
            "namespace:=",
            PathJoinSubstitution([LaunchConfiguration('name'), "gz"])
        ]
    )

    # path = "/home/dexus/ros2_ws/src/ALEX/alex_gazebo/xacro/test.urdf.xacro"
    # robot_description_content = xacro.process_file(path, mappings={"prefix": ""}).toxml()

    robot_description = {"robot_description": robot_description_content}
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("dsr_description2"), "rviz", "default.rviz"]
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        # namespace=PathJoinSubstitution([LaunchConfiguration('name'), "gz"]),
        output="screen",
        parameters=[robot_description],
    )

    # publish fake joint values with GUI
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )


    nodes = [
        joint_state_publisher_node,
        node_robot_state_publisher,
        rviz_node,
    ]

    return LaunchDescription(ARGUMENTS + nodes)

