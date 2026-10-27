#!/usr/bin/env python3
import argparse
import os
import sys

import rclpy
from rclpy.utilities import remove_ros_args

from calibration_tools.joint_state_recorder import JointStatesRecorder


def find_repo_data_dir():
    """Return the `data` folder of the kinematics_calibration repo, or None.

    With `colcon build --symlink-install` this file resolves back into the repo source,
    so the recordings land next to the data used by scripts/run_optimizer.py.
    """
    path = os.path.dirname(os.path.realpath(__file__))
    while path != os.path.dirname(path):
        if os.path.isdir(os.path.join(path, 'calibrate_fk')) and \
                os.path.isdir(os.path.join(path, 'data')):
            return os.path.join(path, 'data')
        path = os.path.dirname(path)
    return None


def default_data_dir():
    if os.environ.get('MUKCA_DATA_DIR'):
        return os.environ['MUKCA_DATA_DIR']
    repo_data_dir = find_repo_data_dir()
    if repo_data_dir is not None:
        return repo_data_dir
    return os.path.join(os.getcwd(), 'data')


def is_writable(path):
    # Check the closest existing parent, since the folder itself may not exist yet
    while not os.path.exists(path):
        path = os.path.dirname(path)
    return os.access(path, os.W_OK)


def main():
    argument_parser = argparse.ArgumentParser(description='Run the joint states recorder')
    argument_parser.add_argument("--joint-state-topic-name", "-j", help="Topic where the joint state is published, usually /joint_states", default="/joint_states")
    argument_parser.add_argument("--robot-name", "-r", help="name of the robot that you are calibrating, e.g. kuka_1", default="robot")
    argument_parser.add_argument("--tool-position-on-table", "-t", help="position of the tool on the table", default="unspecified_tool_position")
    argument_parser.add_argument("--joint-names", "-names", help="List of joint names to record (space-separated). If not specified, all joints from the topic will be used.", nargs='+', required=False, default=None)
    argument_parser.add_argument("--data-dir", "-o", help="Root folder of the datasets. Defaults to $MUKCA_DATA_DIR, else the data folder of this repo, else ./data", default=None)
    argument_parser.add_argument("--input", "-i", help="Keyboard input: 'terminal' reads keys from this terminal, 'pynput' listens globally (needs X)", choices=['terminal', 'pynput'], default='terminal')

    args = argument_parser.parse_args(remove_ros_args(sys.argv)[1:])

    data_dir = os.path.abspath(args.data_dir or default_data_dir())
    data_folder = os.path.join(data_dir, args.robot_name, args.tool_position_on_table)
    if not is_writable(data_folder):
        print(f"Error: {data_folder} is not writable by this user (uid {os.getuid()}).\n"
              f"Pass --data-dir <writable folder> or set MUKCA_DATA_DIR.", file=sys.stderr)
        sys.exit(1)

    print(f"Data folder: {data_folder}")
    print(f"Joint state topic: {args.joint_state_topic_name}")
    print(f"Robot name: {args.robot_name}")
    print(f"Tool position: {args.tool_position_on_table}")
    if args.joint_names is not None:
        print(f"Joint names: {args.joint_names}")
    else:
        print("Joint names: Will use all joints from topic")

    rclpy.init()
    node = JointStatesRecorder(args.joint_state_topic_name, data_folder, args.joint_names, args.input)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
