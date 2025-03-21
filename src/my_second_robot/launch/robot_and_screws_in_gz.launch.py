import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory('my_second_robot')

    # Path to the world file
    world_file = os.path.join(pkg_share, 'worlds', 'empty.sdf')

    # Process the xacro file into a URDF
    robot_desc_path = os.path.join(pkg_share, 'urdf', 'assembly.xacro')
    robot_desc = xacro.process_file(robot_desc_path).toxml()  # Convert xacro to URDF XML

    # Robot State Publisher (publishes the processed URDF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # Gazebo launch with Bullet physics engine (optional)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={
            'gz_args': f'{world_file} -r --physics-engine gz-physics-bullet-featherstone-plugin'
        }.items()
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

    # Controller configuration file
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("my_second_robot"), "config", "ros2_control_config.yaml"]
    )

        # Start ros2_control
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_controllers],
        output='screen'
    )
    
    # Spawning controllers
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_controller"],
        output="screen",
    )


    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )


    # Foxglove Bridge Node
    foxglove_bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="log",
        parameters=[{"port": 8765}],  # Default WebSocket port
    )

    # Command to launch Foxglove Studio
    foxglove_studio = ExecuteProcess(
        cmd=["/opt/Foxglove Studio/foxglove-studio"],
    )



    return LaunchDescription([
        robot_state_publisher,
        
        ros2_control_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        
        
        gazebo,
        spawn_robot,


        clock_bridge,
        foxglove_bridge,
        foxglove_studio,
    ])