import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    candidate_root = os.path.dirname(os.path.dirname(__file__))
    aws_root = '/data/dynaslam/sim_assets/aws-robomaker-small-house-world'
    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    model_path = os.path.join(aws_root, 'models')
    if existing_model_path:
        model_path = model_path + os.pathsep + existing_model_path

    candidate_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(candidate_root, 'launch', 'candidate_world.launch.py')
        ),
        launch_arguments={
            'world': os.path.join(
                candidate_root,
                'worlds',
                'aws_small_house_dynamic_candidate.world',
            ),
            # Central/east-side start with a clear path toward both dynamic areas.
            'robot_x': '2.75',
            'robot_y': '-1.84',
            'robot_yaw': '3.1415926',
            'spawn_box': 'true',
            'box_x': '4.0',
            'box_y': '-3.0',
            'box_yaw': '0.0',
            # Uniform east-side patrol in world X. The box keeps a fixed heading.
            'run_box_patrol': 'true',
            'box_patrol_axis': 'x',
            'box_patrol_lower': '4.0',
            'box_patrol_upper': '6.0',
            'box_patrol_speed': '0.5',
        }.items(),
    )

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path),
        candidate_launch,
    ])
