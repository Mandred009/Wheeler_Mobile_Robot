import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
import xacro


def generate_launch_description():
    robotName='wheeler'
    packageName='robot_description'

    urdfPath="urdf/my_robot.urdf.xacro"
    worldPath="worlds/robo_world_mapper.world"

    rviz_config_path = os.path.join(
        get_package_share_directory(packageName),
        'rviz',
        'rviz_config2.rviz'
    )

    urdfPathAbsolute=os.path.join(get_package_share_directory(packageName),urdfPath)

    worldPathAbsolute=os.path.join(get_package_share_directory(packageName),worldPath)

    robotDescription = xacro.process_file(urdfPathAbsolute).toxml()

    gazebo_ros_package=PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'),'launch','gazebo.launch.py'))

    gazeboLaunch=IncludeLaunchDescription(gazebo_ros_package,launch_arguments={'world':worldPathAbsolute}.items())

    spawnModelNode = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', robotName,
            '-x', '2.0',
            '-y', '0.0',
            '-z', '0.2',
            '-Y', '0.0'  # yaw in radians (Z-axis rotation)
        ]
    )
    
    nodeRobotStatePublisher= Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description':robotDescription,
                     'use_sim_time':True}]
    )

    rviz_node = Node( # Rviz Node
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}]
    )

    static_transform_publisher_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='map_link_broadcaster',
    arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom'],
    output='screen')

    launch_description=LaunchDescription()

    launch_description.add_action(gazeboLaunch)

    launch_description.add_action(spawnModelNode)
    launch_description.add_action(nodeRobotStatePublisher)

    launch_description.add_action(rviz_node)

    # launch_description.add_action(static_transform_publisher_node)

    return launch_description