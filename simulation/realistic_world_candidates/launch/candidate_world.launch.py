import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    world = LaunchConfiguration('world')
    robot_x = LaunchConfiguration('robot_x')
    robot_y = LaunchConfiguration('robot_y')
    robot_yaw = LaunchConfiguration('robot_yaw')
    spawn_box = LaunchConfiguration('spawn_box')
    box_x = LaunchConfiguration('box_x')
    box_y = LaunchConfiguration('box_y')
    box_yaw = LaunchConfiguration('box_yaw')
    run_box_patrol = LaunchConfiguration('run_box_patrol')
    box_patrol_axis = LaunchConfiguration('box_patrol_axis')
    box_patrol_lower = LaunchConfiguration('box_patrol_lower')
    box_patrol_upper = LaunchConfiguration('box_patrol_upper')
    box_patrol_speed = LaunchConfiguration('box_patrol_speed')

    simulation_package = '/home/zhu/ros2_ws/src/dynamic_rtabmap_sim'
    robot_xacro = os.path.join(simulation_package, 'urdf', 'robot.xacro')
    box_sdf = os.path.join(simulation_package, 'models', 'model_box.sdf')
    robot_description = xacro.process_file(robot_xacro).toxml()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py',
            )
        ),
        launch_arguments={
            'world': world,
            'verbose': 'true',
        }.items(),
    )

    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'geo_bot',
            '-x', robot_x,
            '-y', robot_y,
            '-z', '0.30',
            '-Y', robot_yaw,
        ],
        output='screen',
    )

    spawn_dynamic_box = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'dynamic_box',
            '-file', box_sdf,
            '-x', box_x,
            '-y', box_y,
            '-z', '0.30',
            '-Y', box_yaw,
        ],
        output='screen',
        condition=IfCondition(spawn_box),
    )

    box_patrol = ExecuteProcess(
        cmd=[
            'python3',
            os.path.join(
                os.path.dirname(os.path.dirname(__file__)),
                'tools',
                'box_axis_patrol.py',
            ),
            '--axis', box_patrol_axis,
            '--lower', box_patrol_lower,
            '--upper', box_patrol_upper,
            '--speed', box_patrol_speed,
        ],
        output='screen',
    )

    start_box_patrol_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_dynamic_box,
            on_exit=[box_patrol],
        ),
        condition=IfCondition(run_box_patrol),
    )

    return LaunchDescription([
        DeclareLaunchArgument('world'),
        DeclareLaunchArgument('robot_x', default_value='0.0'),
        DeclareLaunchArgument('robot_y', default_value='0.0'),
        DeclareLaunchArgument('robot_yaw', default_value='0.0'),
        DeclareLaunchArgument('spawn_box', default_value='false'),
        DeclareLaunchArgument('box_x', default_value='1.0'),
        DeclareLaunchArgument('box_y', default_value='0.0'),
        DeclareLaunchArgument('box_yaw', default_value='0.0'),
        DeclareLaunchArgument('run_box_patrol', default_value='false'),
        DeclareLaunchArgument('box_patrol_axis', default_value='x'),
        DeclareLaunchArgument('box_patrol_lower', default_value='0.0'),
        DeclareLaunchArgument('box_patrol_upper', default_value='1.0'),
        DeclareLaunchArgument('box_patrol_speed', default_value='0.5'),
        gazebo,
        state_publisher,
        spawn_robot,
        spawn_dynamic_box,
        start_box_patrol_after_spawn,
    ])
