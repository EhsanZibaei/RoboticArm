import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory('my_second_robot')

    world_file = PathJoinSubstitution([
        os.path.join(pkg_share, 'worlds', 'empty.sdf')
    ])

    
    # Path to assembly.xacro
    robot_desc_path = PathJoinSubstitution([
        os.path.join(pkg_share, 'urdf', 'screws.urdf')
    ])
    
    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': world_file}.items()
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command(['xacro ', robot_desc_path])}]
    )
    
    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',  # Correct package name for ROS 2 Jazzy
        executable='create',   # Correct executable for spawning entities
        arguments=[
            '-entity', 'assembly', 
            '-file', robot_desc_path,  # Directly use the path without Command
            '-x', '0.0', '-y', '0.0', '-z', '0.0'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
    ])