import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def load_yaml(package_name, file_path):
    """Load YAML file."""
    package_share_directory = get_package_share_directory(package_name)
    absolute_path_to_file = os.path.join(package_share_directory, file_path)

    with open(absolute_path_to_file, 'r') as file:
        return yaml.safe_load(file)
    
def generate_launch_description():
    package_share_directory = get_package_share_directory('pls_work')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    controllers_yaml = load_yaml('pls_work', 'config/kinematics.yaml')
    load_controllers = Node(
        package='cpp_move_goal',
        executable='hello_moveit',
        # output='screen',
        parameters=[controllers_yaml]  # Load parameters from the YAML file
    )
    return LaunchDescription([
        declare_use_sim_time,
        load_controllers,
        
    ])

    