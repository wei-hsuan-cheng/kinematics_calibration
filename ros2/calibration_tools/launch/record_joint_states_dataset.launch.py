"""Record a calibration dataset with the settings of a yaml file.

Starts record_joint_states_dataset with the settings of config/<config>.yaml (or of any
yaml file given as a path). Each setting can be overridden on the command line.

Usage:
    ros2 launch calibration_tools record_joint_states_dataset.launch.py config:=panda
    ros2 launch calibration_tools record_joint_states_dataset.launch.py config:=panda \
        tool_position_on_table:=left
    ros2 launch calibration_tools record_joint_states_dataset.launch.py config:=/path/to/my_arm.yaml
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# yaml key / launch argument -> record_joint_states_dataset option
OPTIONS = {
    'joint_state_topic_name': '--joint-state-topic-name',
    'robot_name': '--robot-name',
    'tool_position_on_table': '--tool-position-on-table',
    'joint_names': '--joint-names',
    'data_dir': '--data-dir',
    'input': '--input',
}


def resolve_config(config):
    """Return the yaml path for a config name (config/<name>.yaml) or a path."""
    if os.path.isfile(config):
        return config
    name = config[:-len('.yaml')] if config.endswith('.yaml') else config
    path = os.path.join(get_package_share_directory('calibration_tools'), 'config', f'{name}.yaml')
    if not os.path.isfile(path):
        available = sorted(f[:-len('.yaml')] for f in os.listdir(os.path.dirname(path))
                           if f.endswith('.yaml'))
        raise FileNotFoundError(f"Config '{config}' not found. Pass a yaml path or one of {available}")
    return path


def start_recorder(context, *args, **kwargs):
    config_file = resolve_config(LaunchConfiguration('config').perform(context).strip())
    with open(config_file, 'r') as f:
        settings = yaml.safe_load(f) or {}

    unknown = set(settings) - set(OPTIONS)
    if unknown:
        raise ValueError(f"Unknown keys in {config_file}: {sorted(unknown)}. Known: {list(OPTIONS)}")

    # Command-line arguments override the yaml; joint_names is space-separated there.
    for key in OPTIONS:
        override = LaunchConfiguration(key).perform(context).strip()
        if override:
            settings[key] = override.split() if key == 'joint_names' else override

    arguments = []
    for key, option in OPTIONS.items():
        value = settings.get(key)
        if value in (None, '', []):
            continue  # use the recorder's default
        arguments += [option, *map(str, value)] if isinstance(value, list) else [option, str(value)]

    return [Node(
        package='calibration_tools',
        executable='record_joint_states_dataset',
        arguments=arguments,
        output='screen',
        emulate_tty=True,
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value='panda',
                              description='Settings: config/<config>.yaml of this package, or a '
                                          'path to a yaml file'),
        *[DeclareLaunchArgument(key, default_value='',
                                description=f'Override {key} of the yaml (empty: keep it)')
          for key in OPTIONS],
        OpaqueFunction(function=start_recorder),
    ])
