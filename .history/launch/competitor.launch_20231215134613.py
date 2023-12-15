import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration

from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)

from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    # Launch arguments
    trial_name = LaunchConfiguration("trial_name")

    urdf = os.path.join(get_package_share_directory(
        "ariac_description"), "urdf/ariac_robots/ariac_robots.urdf.xacro")

    moveit_config = (
        MoveItConfigsBuilder(
            "ariac_robots", package_name="ariac_moveit_config")
        .robot_description(urdf)
        .robot_description_semantic(file_path="config/ariac_robots.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # Move Group
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ariac_moveit_config"), "/launch",
             "/ariac_robots_moveit.launch.py"]
        )
    )

    # ARIAC_environment
    ariac_environment = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ariac_gazebo"), "/launch", "/ariac.launch.py"]
        ),
        launch_arguments={
            'trial_name': trial_name,
            'competitor_pkg': "nist_competitor",
            'sensor_config': "sensors",
            'dev_mode': "true",
        }.items()
    )

    # Test Competitor node
    test_competitor = Node(
        package="nist_competitor",
        executable="competitor",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
        arguments=['--ros-args', '--log-level', 'move_group_interface:=warn', '--log-level',
                   'moveit_trajectory_processing.time_optimal_trajectory_generation:=error']
    )

    nodes_to_start = [
        test_competitor,
        ariac_environment,
        moveit
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "trial_name", default_value="kitting", description="Name of ariac trial")
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
