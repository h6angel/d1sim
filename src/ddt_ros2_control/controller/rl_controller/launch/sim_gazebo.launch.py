#!/usr/bin/env python
import os
import xacro
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import OpaqueFunction


def launch_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration("robot").perform(context)
    ns = LaunchConfiguration("ns").perform(context)
    world_name = LaunchConfiguration("world").perform(context)

    # Get world file path
    world_file = os.path.join(
        FindPackageShare("gazebo_bridge").find("gazebo_bridge"),
        "worlds",
        world_name,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("gazebo_ros"), "launch"),
                "/gazebo.launch.py",
            ]
        ),
        launch_arguments={
            "world": world_file,
            "pause": "false",
            "verbose": "false",
        }.items(),
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            f"{ns}/robot_description",
            "-entity",
            f"{ns}",
            "-robot_namespace",
            f"{ns}",
            "-x",
            "0.",
            "-y",
            "0.",
            "-z",
            "0.65",
        ],
        output="screen",
    )

    robot_xacro_path = os.path.join(
        get_package_share_directory(robot_name + "_description"),
        "xacro",
        "robot.xacro",
    )

    robot_description = xacro.process_file(
        robot_xacro_path, mappings={"hw_env": "gazebo"}
    ).toxml()

    robot_description = robot_description.replace(
        "package://d1h_description",
        "file://" + get_package_share_directory("d1h_description"),
    )

    robot_description = robot_description.replace(
        get_package_share_directory("gazebo_bridge")+ "/config/controllers.yaml",
        get_package_share_directory("rl_controller")+ "/config/" + robot_name + "/controllers.yaml",
    )
    # print(robot_description)

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": True},
            {"publish_frequency": 15.0},
            {"frame_prefix": ns + "/"},
        ],
        namespace=ns,
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            ns + "/controller_manager",
        ],
    )

    imu_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "imu_sensor_broadcaster",
            "--controller-manager",
            ns + "/controller_manager",
        ],
    )
    rl_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            robot_name + "_rl_controller",
            "--controller-manager",
            ns + "/controller_manager",
        ],
    )

    obstacle_cloud_config = os.path.join(
        get_package_share_directory("gazebo_obstacle_cloud"),
        "config",
        "obstacle_cloud.yaml",
    )
    obstacle_cloud_node = Node(
        package="gazebo_obstacle_cloud",
        executable="obstacle_cloud_node",
        parameters=[
            obstacle_cloud_config,
            {
                "use_sim_time": True,
                "robot_model_name": robot_name + "_description",
            },
        ],
        output="screen",
    )

    nodes = [
        robot_state_pub_node,
        gazebo,
        spawn_entity,
        joint_state_broadcaster_spawner,
        imu_sensor_broadcaster_spawner,
        rl_controller_spawner,
        obstacle_cloud_node,
    ]

    return nodes


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        launch.actions.DeclareLaunchArgument(
            "robot",
            default_value="d1h",
            description="Robot name (d1h)",
        )
    )
    declared_arguments.append(
        launch.actions.DeclareLaunchArgument(
            "world",
            default_value="training_arena.world",
            description="Gazebo world file under gazebo_bridge/worlds",
        )
    )
    declared_arguments.append(
        launch.actions.DeclareLaunchArgument(
            "ns",
            default_value="",
            description="Namespace of launch",
        )
    )
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
