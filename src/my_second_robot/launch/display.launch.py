import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_share = get_package_share_directory('my_second_robot')
    urdf_path = os.path.join(pkg_share, 'urdf', 'robot_w_screwdriver.urdf')

    with open(urdf_path, 'r') as file:
        robot_description = file.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        name='irb6640_205',
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_publisher_node = Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            output="screen"
        )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node
    ])
