import os
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_share_directory = get_package_share_directory('hubert_group_moveit')
    moveit_config = MoveItConfigsBuilder("hubert", package_name="hubert_group_moveit").to_moveit_configs()
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    controllers_yaml = os.path.join(package_share_directory, 'config', 'kinematics.yaml')
    load_controllers = Node(
        # package='controller_manager',
        executable='spawner',
        # arguments=['--controller-manager', '/controller_manager'],
        # output='screen',
        parameters=[controllers_yaml]  # Load parameters from the YAML file
    )
    demo_launch = generate_demo_launch(moveit_config)
    return LaunchDescription([
        declare_use_sim_time,
        # load_controllers,
        demo_launch,
    ])

    