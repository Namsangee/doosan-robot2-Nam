# 
#  dsr_bringup2
#  Author: Minsoo Song (minsoo.song@doosan.com)
#  
#  Copyright (c) 2024 Doosan Robotics
#  Use of this source code is governed by the BSD, see LICENSE
# 

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, RegisterEventHandler, LogInfo, TimerAction, OpaqueFunction
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def rviz_node_function(context):
    model_value = LaunchConfiguration('model').perform(context)
    package_name = f"dsr_moveit_config_{model_value}"
    package_path = FindPackageShare(package_name).perform(context)
    print("[INFO] MoveIt Config Package:", package_name)
    print("[INFO] Package Path:", package_path)

    moveit_config = (
        MoveItConfigsBuilder(model_value, "robot_description", package_name)
        .robot_description(file_path=f"config/{model_value}.urdf.xacro")
        .robot_description_semantic(file_path="config/dsr.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    rviz_full_config = os.path.join(package_path, "launch", "moveit.rviz")

    return [
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[moveit_config.to_dict()],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_full_config],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.planning_pipelines,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
            ],
        ),
    ]

def generate_launch_description():
    ARGUMENTS = [
        DeclareLaunchArgument('name',  default_value='', description='NAME_SPACE'),
        DeclareLaunchArgument('host',  default_value='127.0.0.1', description='ROBOT_IP'),
        DeclareLaunchArgument('port',  default_value='12345', description='ROBOT_PORT'),
        DeclareLaunchArgument('mode',  default_value='virtual', description='OPERATION MODE'),
        DeclareLaunchArgument('model', default_value='m0617', description='ROBOT_MODEL'),
        DeclareLaunchArgument('color', default_value='white', description='ROBOT_COLOR'),
        DeclareLaunchArgument('gui',   default_value='false', description='Start RViz2'),
        DeclareLaunchArgument('gz',    default_value='false', description='USE GAZEBO SIM'),
        DeclareLaunchArgument('rt_host', default_value='192.168.137.50', description='ROBOT_RT_IP'),
    ]

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare("dsr_description2"), "xacro", LaunchConfiguration('model')]),
        ".urdf.xacro",
        " name:=", LaunchConfiguration('name'),
        " host:=", LaunchConfiguration('host'),
        " rt_host:=", LaunchConfiguration('rt_host'),
        " port:=", LaunchConfiguration('port'),
        " mode:=", LaunchConfiguration('mode'),
        " model:=", LaunchConfiguration('model'),
    ])
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution([
        FindPackageShare("dsr_controller2"),
        "config", "dsr_controller2.yaml",
    ])

    run_emulator_node = Node(
        package="dsr_bringup2",
        executable="run_emulator",
        namespace=LaunchConfiguration('name'),
        parameters=[{
            "name": LaunchConfiguration('name'),
            "rate": 100,
            "standby": 5000,
            "command": True,
            "host": LaunchConfiguration('host'),
            "port": LaunchConfiguration('port'),
            "mode": LaunchConfiguration('mode'),
            "model": LaunchConfiguration('model'),
            "gripper": "none",
            "mobile": "none",
            "rt_host": LaunchConfiguration('rt_host'),
        }],
        output="screen",
    )

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=LaunchConfiguration('name'),
        output='both',
        parameters=[robot_description],
    )

    control_node = TimerAction(
        period=2.0,
        actions=[
            LogInfo(msg=">> [STEP 2] Launching controller_manager (ros2_control_node)..."),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                namespace=LaunchConfiguration('name'),
                parameters=[robot_description, robot_controllers],
                output="both",
            ),
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration('name'),
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration('name'),
        executable="spawner",
        arguments=["dsr_controller2", "-c", "controller_manager"],
    )

    dsr_moveit_controller_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration('name'),
        executable="spawner",
        arguments=["dsr_moveit_controller", "-c", "controller_manager"],
    )

    rviz_node = OpaqueFunction(function=rviz_node_function)

    delay_joint_state_after_control = TimerAction(
        period=3.0,
        actions=[
            LogInfo(msg=">> [STEP 3] Spawning joint_state_broadcaster..."),
            joint_state_broadcaster_spawner,
        ],
    )

    delay_robot_controller_after_joint_state = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                LogInfo(msg=">> [STEP 4] joint_state_broadcaster active. Launching dsr_controller2..."),
                robot_controller_spawner,
            ],
        )
    )

    delay_moveit_controller_after_robot_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[
                LogInfo(msg=">> [STEP 5] dsr_controller2 active. Launching dsr_moveit_controller..."),
                dsr_moveit_controller_spawner,
            ],
        )
    )

    delay_rviz_after_moveit_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=dsr_moveit_controller_spawner,
            on_exit=[
                LogInfo(msg=">> [STEP 6] MoveIt controller active. Launching MoveGroup + RViz..."),
                rviz_node,
            ],
        )
    )

    nodes = [
        run_emulator_node,
        robot_state_pub_node,
        control_node,
        delay_joint_state_after_control,
        delay_robot_controller_after_joint_state,
        delay_moveit_controller_after_robot_controller,
        delay_rviz_after_moveit_controller,
    ]

    return LaunchDescription(ARGUMENTS + nodes)
