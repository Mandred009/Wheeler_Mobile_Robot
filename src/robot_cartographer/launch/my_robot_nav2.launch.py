import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
import xacro


def generate_launch_description():
    package_name = 'robot_cartographer'
    localization_config_path = 'config/localization.yaml'
    nav2_config_path = 'config/nav2.yaml'
    map_path='maps/my_map.yaml'

    localization_config_path_absolute = os.path.join(
        get_package_share_directory(package_name),
        localization_config_path
    )
    nav2_config_path_absolute = os.path.join(
        get_package_share_directory(package_name),
        nav2_config_path
    )

    map_path_absolute = os.path.join(
        get_package_share_directory(package_name),
        map_path
    )
    

    nav2_ros_localization_package=PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav2_bringup'),'launch','localization_launch.py'))
    nav2_localization_launch=IncludeLaunchDescription(nav2_ros_localization_package,launch_arguments={
                'map': map_path_absolute,
                'use_sim_time': "True",
                'params_file': localization_config_path_absolute}.items())
    
    nav2_ros_nav_package=PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav2_bringup'),'launch','navigation_launch.py'))
    nav2_nav_launch=IncludeLaunchDescription(nav2_ros_nav_package,launch_arguments={
                'use_sim_time': "True",
                'use_composition': "False",
                'params_file': nav2_config_path_absolute}.items())
    
    

    launch_description=LaunchDescription()

    launch_description.add_action(nav2_localization_launch)
    launch_description.add_action(nav2_nav_launch)
    

    return launch_description