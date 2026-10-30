"""Optional: export a robot's urdf.xacro as the plain urdf the calibration reads.

The calibration scripts read a plain urdf from urdf/<arm_name>.urdf. For a robot described by a
.urdf.xacro, this expands the xacro set below, writes
urdf/<arm_name>.urdf and exits. The workspace that provides the description package must be
sourced, so xacro can resolve its $(find ...) includes.

Usage (edit the defaults below for your robot, or override them on the command line):
    ros2 launch calibration_tools export_urdf.launch.py
        -> <kinematics_calibration>/urdf/<arm_name>.urdf, then use --model <arm_name>
"""

import os
import subprocess

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration

# Set these for your robot.
# Package holding the description, as a package name or a path to it.
DESCRIPTION_PACKAGE = 'my_robot_description'
# The .urdf.xacro, relative to the package (an absolute path is used as is).
XACRO_FILE = 'urdf/my_robot.urdf.xacro'
# xacro arguments, space-separated name:=value. Only those that change link and joint
# names (e.g. a name prefix) matter; arguments that only select hardware or simulation
# do not change the kinematics.
XACRO_ARGS = ''
# Written to urdf/<arm_name>.urdf; use --model <arm_name> in the calibration scripts.
ARM_NAME = 'my_arm'


def find_repo_urdf_dir():
    """Return the `urdf` folder of the kinematics_calibration repo, or None.

    Same lookup as record_joint_states_dataset.find_repo_data_dir, so the exported urdf
    lands where scripts/run_optimizer.py looks for --model.
    """
    path = os.path.dirname(os.path.realpath(__file__))
    while path != os.path.dirname(path):
        if os.path.isdir(os.path.join(path, 'calibrate_fk')) and \
                os.path.isdir(os.path.join(path, 'urdf')):
            return os.path.join(path, 'urdf')
        path = os.path.dirname(path)
    return None


def export_urdf(context, *args, **kwargs):
    package = LaunchConfiguration('description_package').perform(context).strip()
    xacro_file = LaunchConfiguration('xacro_file').perform(context).strip()
    xacro_args = LaunchConfiguration('xacro_args').perform(context).split()
    arm_name = LaunchConfiguration('arm_name').perform(context).strip()

    if not os.path.isabs(xacro_file):
        if os.path.isdir(package):
            package_dir = package
        else:
            try:
                package_dir = get_package_share_directory(package)
            except PackageNotFoundError:
                raise RuntimeError(
                    f"Description package '{package}' not found. Set DESCRIPTION_PACKAGE in "
                    "export_urdf.launch.py (or description_package:=...) and source its "
                    "workspace.")
        xacro_file = os.path.join(package_dir, xacro_file)
    if not os.path.isfile(xacro_file):
        raise FileNotFoundError(f"{xacro_file} not found. Set XACRO_FILE in export_urdf.launch.py "
                                "(or xacro_file:=...).")

    urdf_dir = find_repo_urdf_dir() or os.path.join(os.getcwd(), 'urdf')
    output_file = os.path.join(urdf_dir, f'{arm_name}.urdf')

    result = subprocess.run(['xacro', xacro_file, *xacro_args], capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(f"xacro failed for {xacro_file}:\n{result.stderr}")

    os.makedirs(urdf_dir, exist_ok=True)
    with open(output_file, 'w') as f:
        f.write(result.stdout)
    return [LogInfo(msg=f"[export_urdf] {xacro_file} -> {output_file}")]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('description_package', default_value=DESCRIPTION_PACKAGE,
                              description='Package holding the description (name or path)'),
        DeclareLaunchArgument('xacro_file', default_value=XACRO_FILE,
                              description='urdf.xacro, relative to the package or absolute'),
        DeclareLaunchArgument('xacro_args', default_value=XACRO_ARGS,
                              description='xacro arguments, space-separated name:=value'),
        DeclareLaunchArgument('arm_name', default_value=ARM_NAME,
                              description='Arm name: writes urdf/<arm_name>.urdf'),
        OpaqueFunction(function=export_urdf),
    ])
