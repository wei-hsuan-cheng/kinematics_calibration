# calibration_tools (ROS 2 Humble)

Records the joint configurations of the robot while the ball is placed in socket 0 and socket 1 of the MUKCa tool.
The recorded csv files are the input of `scripts/run_optimizer.py`.

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

Dependencies: `rclpy`, `sensor_msgs`, `numpy`. `pynput` is only needed for `--input pynput`.

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
