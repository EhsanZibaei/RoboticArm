import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory('my_second_robot')

    # Path to the world file
    world_file = os.path.join(pkg_share, 'worlds', 'empty.sdf')

    # Process the xacro file into a URDF
    robot_desc_path = os.path.join(pkg_share, 'urdf', 'assembly.xacro')
    robot_desc = xacro.process_file(robot_desc_path).toxml()  # Convert xacro to URDF XML

    # Gazebo launch with Bullet physics engine (optional)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={
            'gz_args': f'{world_file} -r --physics-engine gz-physics-bullet-featherstone-plugin'
        }.items()
    )

    # Robot State Publisher (publishes the processed URDF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # Spawn robot in Gazebo using the processed URDF
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-entity', 'assembly',        # Name of the entity in Gazebo
            '-string', robot_desc,       # Pass the processed URDF as a string
            '-x', '0.0', '-y', '0.0', '-z', '0.1'  # Slightly above ground to avoid collision
        ],
        output='screen'
    )

    return LaunchDescription([
        # robot_state_publisher,
        spawn_robot,
        gazebo,
    ])