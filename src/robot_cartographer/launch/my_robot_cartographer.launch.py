import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
import xacro


def generate_launch_description():
    package_name = 'robot_cartographer'
    config_path = 'config/my_robot_carto.lua'

    config_path_absolute = os.path.join(
        get_package_share_directory(package_name),
        config_path
    )

    config_dir = os.path.dirname(config_path_absolute)
    config_basename = os.path.basename(config_path_absolute)

    carto_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basename', config_basename
        ]
    )

    occupancy_node=Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['-resolution', "0.05", '-publish_period_sec', "1.0"])

    launch_description=LaunchDescription()

    launch_description.add_action(carto_node)
    launch_description.add_action(occupancy_node)

    return launch_description