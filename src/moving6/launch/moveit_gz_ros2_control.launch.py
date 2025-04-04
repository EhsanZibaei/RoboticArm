# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    gz_args = LaunchConfiguration('gz_args', default='')

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('moving6'),
                 'config', 'circu.urdf.xacro']
            ),
        ]
    )
    robot_description = {'robot_description': robot_description_content}
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('moving6'),
            'config',
            'ros2_controllers.yaml',
        ]
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'cart', '-allow_renaming', 'true'],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   ],
    )
    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'g1_controller',
            '--param-file',
            robot_controllers,
            ],
    )

    screwdriver_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'screwdriver_controller',
            '--param-file',
            robot_controllers,
            ],
    )
    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="both",
    )

    launch_arguments = {
        "robot_ip": "xxx.yyy.zzz.www",
        "use_fake_hardware": "false",
        "gripper": "robotiq_2f_85",
        "dof": "7",
    }

    # Load the robot configuration
    moveit_config = (
        MoveItConfigsBuilder(
            "gen3", package_name="moving6"
        )
        .robot_description(mappings=launch_arguments)
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=False, publish_robot_description_semantic=False
        )
        .planning_pipelines(
            pipelines=["ompl", "stomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        # arguments=["--ros-args", "--log-level", "debug"],
        parameters=[moveit_config.to_dict(),
                    {'use_sim_time': use_sim_time},
                    ],
    )

    # RViz
    rviz_config_file = (
        get_package_share_directory("moving6") + "/config/moveit.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
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
        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', [gz_args, 'empty.sdf -r --physics-engine gz-physics-bullet-featherstone-plugin'])]),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[joint_trajectory_controller_spawner],
            )
        ),
        ros2_control_node,
        bridge,
        node_robot_state_publisher,
        gz_spawn_entity,
        run_move_group_node,
        rviz_node,
        foxglove_bridge,
        foxglove_studio,
        screwdriver_controller_spawner,
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
    ])



