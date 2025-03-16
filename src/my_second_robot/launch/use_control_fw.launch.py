import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare




def generate_launch_description():
    pkg_share = get_package_share_directory('my_second_robot')
    urdf_path = os.path.join(pkg_share, 'urdf', 'robot_w_screwdriver.urdf')

    with open(urdf_path, 'r') as file:
        robot_description = file.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
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

    # Delay start of joint_state_broadcaster after `robot_controller`
    # delay_robot_controller_after_joint_state_broadcaster = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=joint_state_broadcaster_spawner,
    #         on_exit=[robot_controller_spawner],
    #     )
    # )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", PathJoinSubstitution(
        [FindPackageShare("my_second_robot"), "config", "rviz_config.rviz"])],
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
        robot_state_publisher_node,
        # joint_state_publisher_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        # delay_robot_controller_after_joint_state_broadcaster,
        foxglove_bridge,
        foxglove_studio,
        rviz_node,
    ])


