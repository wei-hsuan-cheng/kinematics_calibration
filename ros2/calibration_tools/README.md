# calibration_tools (ROS 2 Humble)

Records the joint configurations of the robot while the ball is placed in socket 0 and socket 1 of the MUKCa tool,
with the settings of a yaml file per robot. The recorded csv files are the input of `scripts/run_optimizer.py`.

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

Dependencies: `rclpy`, `sensor_msgs`, `numpy`; the launch files also use `launch`, `launch_ros` and `yaml`. `pynput` is only needed for `input: pynput`. The urdf export also uses `xacro`, and needs the workspace with the description package sourced.

## Usage

The recorder settings live in a yaml file per robot, in [`config`](config):

```bash
ros2 launch calibration_tools record_joint_states_dataset.launch.py config:=panda
```

`config:=<name>` loads `config/<name>.yaml`; `config:=/path/to/my_arm.yaml` loads any yaml file.
Each setting can be overridden on the command line, e.g. when the tool is moved:

```bash
ros2 launch calibration_tools record_joint_states_dataset.launch.py config:=panda tool_position_on_table:=left
```

| yaml key / launch argument | Recorder default (key empty or missing) | Description |
|---|---|---|
| `joint_state_topic_name` | `/joint_states` | `sensor_msgs/msg/JointState` topic |
| `robot_name` | `robot` | Robot id, e.g. `panda_1`, `kuka_14`, `kinova_gen3` |
| `tool_position_on_table` | `unspecified_tool_position` | Position of the tool, e.g. `front`, `left` |
| `joint_names` | all joints of the first message | Joints to record, in this order (a yaml list; space-separated on the command line) |
| `data_dir` | see [Data storage](#data-storage) | Root folder of the datasets |
| `input` | `terminal` | `tty`: keys typed in this terminal; use it under `ros2 launch` (the configs set it). `pynput`: global key listener, needs X and `pip3 install pynput` |

Provided configs, for the robots in the `urdf` folder:

| `config:=` | Robot | `robot_name` | `joint_names` |
|---|---|---|---|
| `panda` | Franka Panda / FR3 | `panda_1` | `panda_joint1` ... `panda_joint7` |
| `iiwa14` | KUKA iiwa14 | `kuka_1` | `iiwa_joint_1` ... `iiwa_joint_7` |
| `gen3lite` | Kinova Gen3 Lite | `kinova_gen3` | `joint_1` ... `joint_6` |
| `panda_plus_bota` | Franka Panda with a Bota sensor | `panda_with_sensor` | `panda_joint1` ... `panda_joint7` |

For another robot, copy one of them, set `robot_name` and `joint_names`, and pass its path with `config:=`.

### Direct `ros2 run`

The launch file starts this executable with the matching options, which can also be passed directly:

```bash
ros2 run calibration_tools record_joint_states_dataset \
  --joint-state-topic-name /joint_states \
  --robot-name panda_1 \
  --tool-position-on-table front \
  --joint-names panda_joint1 panda_joint2 panda_joint3 panda_joint4 panda_joint5 panda_joint6 panda_joint7
```

| Option | yaml key |
|---|---|
| `--joint-state-topic-name`, `-j` | `joint_state_topic_name` |
| `--robot-name`, `-r` | `robot_name` |
| `--tool-position-on-table`, `-t` | `tool_position_on_table` |
| `--joint-names`, `-names` | `joint_names` |
| `--data-dir`, `-o` | `data_dir` |
| `--input`, `-i` | `input`: `terminal` reads keys from stdin (default for `ros2 run`, and allows piping keys in), `tty` from the controlling terminal, `pynput` globally |

`ros2 launch` does not pass the terminal to its processes as stdin, which is why the configs use `input: tty`.

Messages that do not contain all requested joints are ignored, so a data point is never recorded with missing joints.

### Key controls

- `a` add a data point to the active hole
- `d` delete the last data point of the active hole
- `s` switch between hole 0 and hole 1
- `i` show the help
- `q` save the data and quit

## Data storage

The data is saved in `<data-dir>/<robot-name>/<tool-position-on-table>/hole_{0,1}.csv`, where `<data-dir>` is

1. `data_dir` (yaml key or `data_dir:=`; `--data-dir` with `ros2 run`), if set
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
Either record to a writable folder with `data_dir:=<folder>` (`--data-dir` with `ros2 run`) and copy the result to `data/` afterwards, or give the container user write access on the host, e.g. `setfacl -R -m u:<container uid>:rwX -m d:u:<container uid>:rwX data calibrated_urdf`.

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
