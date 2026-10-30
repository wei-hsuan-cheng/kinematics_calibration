# calibration_tools (ROS 2 Humble)

Records the joint configurations of the robot while the ball is placed in socket 0 and socket 1 of the MUKCa tool.
The recorded csv files are the input of `scripts/run_optimizer.py`.

Optionally, it also [exports a `.urdf.xacro`](#export-a-urdf-from-xacro-optional) as the plain urdf the calibration reads.

## Build

Symlink the whole `kinematics_calibration` repository into a colcon workspace; colcon finds this package in `ros2/calibration_tools`.

```bash
cd <workspace_dir>
ln -s <path/to/kinematics_calibration> src/
NUM_JOBS=2 && \
export CMAKE_BUILD_PARALLEL_LEVEL=${NUM_JOBS} && \
export MAKEFLAGS=-j${NUM_JOBS} && \
export NINJAFLAGS=-j${NUM_JOBS} && \
colcon build --symlink-install \
  --packages-up-to calibration_tools \
  --executor sequential --parallel-workers ${NUM_JOBS} \
  --cmake-force-configure \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release && \
  . install/setup.bash
```

Run the tests with

```bash
colcon test --packages-select calibration_tools && colcon test-result --verbose
```

Dependencies: `rclpy`, `sensor_msgs`, `numpy`. `pynput` is only needed for `--input pynput`. The urdf export also uses `launch` and `xacro`, and needs the workspace with the description package sourced.

## Usage

```bash
ros2 run calibration_tools record_joint_states_dataset \
  --joint-state-topic-name /joint_states \
  --robot-name panda_1 \
  --tool-position-on-table front \
  --joint-names panda_joint1 panda_joint2 panda_joint3 panda_joint4 panda_joint5 panda_joint6 panda_joint7
```

| Argument | Default | Description |
|---|---|---|
| `--joint-state-topic-name`, `-j` | `/joint_states` | `sensor_msgs/msg/JointState` topic |
| `--robot-name`, `-r` | `robot` | Robot id, e.g. `panda_1`, `kuka_14`, `kinova_gen3` |
| `--tool-position-on-table`, `-t` | `unspecified_tool_position` | Position of the tool, e.g. `front`, `left` |
| `--joint-names`, `-names` | all joints of the first message | Joints to record, in this order |
| `--data-dir`, `-o` | see below | Root folder of the datasets |
| `--input`, `-i` | `terminal` | `terminal`: keys typed in this terminal (works in docker without X). `pynput`: global key listener, needs X and `pip3 install pynput` |

Joint names for the robots in the `urdf` folder:

- Franka Panda / FR3: `panda_joint1` ... `panda_joint7`
- KUKA iiwa14: `iiwa_joint_1` ... `iiwa_joint_7`
- Kinova Gen3 Lite: `joint_1` ... `joint_6`

Messages that do not contain all requested joints are ignored, so a data point is never recorded with missing joints.

### Key controls

- `a` add a data point to the active hole
- `d` delete the last data point of the active hole
- `s` switch between hole 0 and hole 1
- `i` show the help
- `q` save the data and quit

## Data storage

The data is saved in `<data-dir>/<robot-name>/<tool-position-on-table>/hole_{0,1}.csv`, where `<data-dir>` is

1. `--data-dir`, if given
2. `$MUKCA_DATA_DIR`, if set
3. the `data` folder of this repository (found through the `--symlink-install` symlinks), so that `run_optimizer.py --data <robot-name>/<tool-position-on-table>` finds it
4. `./data`

If the folder already contains recordings, the new data points are appended to them.

The joint names are stored as a `#` header, which is skipped by `np.loadtxt` and by `calibrate_fk`:

```csv
#panda_joint1,panda_joint2,panda_joint3,panda_joint4,panda_joint5,panda_joint6,panda_joint7
1.230000000000000000e-01,-4.560000000000000000e-01,...
```

### Docker

If the repository is mounted into a container that runs with a different uid than the owner of the files, the `data` folder is not writable from the container and the recorder exits with an error.
Either record to a writable folder with `--data-dir` and copy the result to `data/` afterwards, or give the container user write access on the host, e.g. `setfacl -R -m u:<container uid>:rwX -m d:u:<container uid>:rwX data calibrated_urdf`.

## Export a urdf from xacro (optional)

The calibration reads a plain urdf (`urdf/<arm_name>.urdf`). If your robot is described by a `.urdf.xacro`,
[`launch/export_urdf.launch.py`](launch/export_urdf.launch.py) expands
the xacro set at its top, writes the plain urdf and exits; no robot has to be running. The workspace that
provides the description package must be sourced, so xacro can resolve its `$(find ...)` includes.

```bash
source <workspace_dir>/install/setup.bash
ros2 launch calibration_tools export_urdf.launch.py
# -> <kinematics_calibration>/urdf/<arm_name>.urdf, then use --model <arm_name>
```

Set these at the top of the launch file for your robot (each can also be overridden on the command line):

| Setting / argument | Default | Description |
|---|---|---|
| `DESCRIPTION_PACKAGE` / `description_package` | `my_robot_description` | Package holding the description, as a name or a path |
| `XACRO_FILE` / `xacro_file` | `urdf/my_robot.urdf.xacro` | The `.urdf.xacro`, relative to the package (absolute paths are used as is) |
| `XACRO_ARGS` / `xacro_args` | empty | xacro arguments, space-separated `name:=value` |
| `ARM_NAME` / `arm_name` | `my_arm` | Writes `urdf/<arm_name>.urdf`; use `--model <arm_name>` in the scripts |

Only the xacro arguments that change link and joint names (e.g. a name prefix) matter. Arguments that only
select hardware or simulation do not change the kinematics.

The `urdf` folder is found like the [data folder](#data-storage), through the `--symlink-install` symlinks;
otherwise `./urdf` is used.

The exported file is the robot only. Add the [sphere](../../README.md#add-the-sphere-to-the-urdf) and a
`config_optimizer/<arm_name>.yaml` before calibrating.
