# MUKCa: Minimal User-friendly Kinematics Calibration of Robot Manipulators

The repo is forked from [`platonics-delft/kinematics_calibration`](https://github.com/platonics-delft/kinematics_calibration).

<table align="center">
  <tr>
    <td align="center" width="50%">
      <img src="imgs/mukca.png" width="450" alt="MUKCa calibration tool" />
    </td>
    <td align="center" width="50%">
      <img src="imgs/Calibration_scheme.png" width="300" alt="MUKCa calibration scheme" />
    </td>
  </tr>
  <tr>
    <td align="center"><b>MUKCa tool</b><br />Sockets 0 and 1, 50 mm apart, and the sphere attached to the end effector</td>
    <td align="center"><b>Calibration scheme</b><br />Consistency in each socket and distortion of the distance between them</td>
  </tr>
</table>


## Installation


### Recorder (ROS 2 Humble)
```bash
cd <workspace_dir>
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


### Calibration (Python, no ROS)
Install the python package `calibrate_fk` in a virtual environment, so that pip does not replace the numpy, scipy and matplotlib used by ROS:
```bash
python3 -m virtualenv ~/.venvs/mukca   # or: python3 -m venv ~/.venvs/mukca, or: uv venv -p 3.10 ~/.venvs/mukca
~/.venvs/mukca/bin/pip install -e <path/to/kinematics_calibration>
```
Or install it through poetry with `poetry install`. All dependencies are available for x86_64 and aarch64 (e.g. Jetson with JetPack 6), Python 3.10. Create the environment on each machine, do not copy it.


## Overview
The calibration has two independent steps that share only the recorded csv files:

| Step | What | Environment |
|---|---|---|
| 1. Record | ROS 2 package [`ros2/calibration_tools`](ros2/calibration_tools) saves the joint configurations with the ball in socket 0 and socket 1 to `data/<robot-name>/<tool-position-on-table>/hole_{0,1}.csv` | ROS 2 Humble |
| 2. Calibrate | Python package `calibrate_fk` and the `scripts` folder optimize the URDF on the recorded data and write it to `calibrated_urdf/<robot-name>/` | Python virtual environment, no ROS |


## Record the data

<p align="center">
  <img src="imgs/mukca_franka.gif" width="500" alt="Recording data with a Franka" />
  <br />
  <b>Franka Panda</b><br />Recording joint configurations with the sphere in the sockets
</p>

The tool has two sockets identified as 0 and 1. There is a sphere attached at the end effector. The idea is to record many joint configurations with the ball in socket 0 and many others with the ball in socket 1, at least 20 for each socket.

1. Start the driver or controller of your robot, and check that it publishes a `sensor_msgs/msg/JointState` topic:
   ```bash
   ros2 topic echo /joint_states --once
   ```
2. Run the recorder, e.g. for a Franka Panda:
   ```bash
   ros2 run calibration_tools record_joint_states_dataset \
     --joint-state-topic-name /joint_states \
     --robot-name panda_1 \
     --tool-position-on-table front \
     --joint-names panda_joint1 panda_joint2 panda_joint3 panda_joint4 panda_joint5 panda_joint6 panda_joint7
   ```
3. Type the keys in the terminal running the recorder (no Enter needed):
   - Press `a` to add a new joint configuration with the sphere in the socket that is displayed in green.
   - Press `d` to delete the last joint configuration from the list.
   - Press `s` when you switch the socket in which you are recording.
   - Press `i` to show the help.
   - Press `q` when you have finished recording, to save the data and quit.
4. Optionally, place the tool in other parts of the workspace and record again with the same `--robot-name` and a different `--tool-position-on-table`, e.g. `left`, `right`, `high`.

The data is saved in `data/<robot-name>/<tool-position-on-table>/hole_0.csv` and `hole_1.csv`, with the joint names as a `#` header. If the folder already contains recordings, the new data points are appended.

- `--robot-name`: use the name of the robot and a number, in case you want to calibrate more than one robot of the same type. All datasets of one robot must use the same robot name, since the evaluation scripts look for the test data in `data/<robot-name>`.
- `--tool-position-on-table`: only a label to distinguish the datasets; the pose of the tool is never used. Use a new name every time the tool is moved, since all the data in one folder must share the same two socket locations.
- `--joint-names`: the joints to record, in this order. If omitted, all joints of the first message are recorded. Messages that do not contain all the joints are ignored.

| Robot | `--model` | `--joint-names` |
|---|---|---|
| Franka Panda / FR3 | `panda` | `panda_joint1` ... `panda_joint7` |
| KUKA iiwa14 | `iiwa14` | `iiwa_joint_1` ... `iiwa_joint_7` |
| Kinova Gen3 Lite | `gen3lite` | `joint_1` ... `joint_6` |

See [`ros2/calibration_tools/README.md`](ros2/calibration_tools/README.md) for all options, e.g. `--data-dir` to save the data somewhere else.


## Calibrate the urdf model
Activate the virtual environment and go to the `scripts` folder:
```bash
source ~/.venvs/mukca/bin/activate
cd <path/to/kinematics_calibration>/scripts
export MPLBACKEND=Agg   # optional: save the plots to files instead of opening windows
```

Three scripts are run in sequence. Only the first one changes the model; the other two evaluate its result and can be run in any order afterwards. `generate_complete_analysis.py` runs all of them, including the overlay below.

| # | Script | What it does | Output |
|---|---|---|---|
| 1 | `run_optimizer.py` | Optimizes the `xyz` `rpy` of the joints listed in `config_optimizer/<model>.yaml`, such that the predicted ball position is consistent in each socket and the two sockets are 50 mm apart | `calibrated_urdf/<robot-name>/<model>.urdf`, `config.yaml` and the model after each optimization step `step_*/model.urdf` |
| 2 | `generate_learning_curve.py` | Evaluates every optimization step on the training data and on all the other datasets in `data/<robot-name>` | `consistency.png`, `distortion.png` |
| 3 | `compute_improved_performance.py` | Compares the nominal and the calibrated model on the training data and on the other datasets, and prints the parameter changes | Terminal output |


### 1. Optimize
```bash
python3 run_optimizer.py --model <nominal_urdf_name> --data <robot-name>/<tool-position-on-table>
```
The nominal model needs to be in the `urdf` folder, e.g. `panda` for `urdf/panda.urdf`. The data are the ones saved in the previous step, e.g. `panda_1/front`. The script prints the error before and after the optimization, and writes a new urdf with the same name as the original one to the `calibrated_urdf` folder, in the subfolder of the robot name. So `calibrated_urdf/panda_1/panda.urdf` contains the calibrated parameters. Running the script again overwrites this folder.
```bash
python3 run_optimizer.py --model panda --data panda_1/front
```
If the printed tool is not exactly 50 mm, pass the distance measured with a caliper in meters with `--offset-distance`, see [Print the tool](#print-the-tool).

Training on one position is sufficient, but training on multiple tool positions that are placed in different parts of the workspace is as easy as specifying their paths in the `--data` input. All the paths must belong to the same robot:
```bash
python3 run_optimizer.py --model panda --data panda_1/front panda_1/left panda_1/right panda_1/high
```
The optimization will now try to maximize the consistency in any of the sockets of each tool position and minimize the bias in the predicted distance between the two sockets.

<p align="center">
  <img src="imgs/bar_plot_panda_2.jpg" width="800" alt="Mean absolute error for different training and test positions of the Panda" />
  <br />
  <b>Franka Panda</b><br />Mean absolute error when training on one, several or all tool positions (hatched: training)
</p>

In this plot we trained and evaluated on different positions of the tool or in all of them. You can see that even when training only on the front, the final accuracy on the right-high position is still good. By training on all the positions, the error goes down everywhere.


### 2. Plot the learning curves
```bash
python3 generate_learning_curve.py --model panda_1
```
This plots the consistency and the distortion of the model both on the training set and on any other dataset in the folder `data/panda_1`, and saves them in `calibrated_urdf/panda_1`.
For example:
<table align="center">
  <tr>
    <td align="center" width="50%">
      <img src="imgs/consistency.png" width="400" alt="Consistency learning curve" />
    </td>
    <td align="center" width="50%">
      <img src="imgs/distortion.png" width="400" alt="Distortion learning curve" />
    </td>
  </tr>
  <tr>
    <td align="center"><b>Consistency</b><br />Variance of the predicted sphere position in each socket</td>
    <td align="center"><b>Distortion</b><br />Error of the predicted distance between the two sockets</td>
  </tr>
</table>

The left curve is the consistency, i.e. the variance of the different forward kinematics positions for each of the two holes. The optimization is trying to make the robot as consistent and as little distorted as possible, that is why you see the curve go down.
The right figure shows the distortion of the robot, i.e. the error in the distance between the two sets of points for socket 0 and socket 1. We expect the distance to be 0.05 m, so we can compute the error of our measurement.


### 3. Evaluate the calibrated model
```bash
python3 compute_improved_performance.py --model panda_1
```
The terminal will output the following statistics, e.g. for `panda_1` trained on `front_center`:
```
The mean absolute error  went from 6.78e-03 to 1.65e-04 on the train data on average
Percentage of removed error on train set: 97.56
The mean absolute error  went from 7.94e-03 to 3.46e-04 on the test data on average
Percentage of removed error on test set: 95.03
```
On the training set but also on the test set, i.e. the tool positions that were not used for training, 95 % of the consistency error was removed after the calibration. This makes the robot almost perfectly calibrated. If all the datasets of the robot are used for training, there is no test set and only the training error is printed; keep one tool position out of the training to check how well the calibration generalizes.


### Visualize the overlay of the robot prediction
You can also generate an overlay of the robot in the different configurations that were recorded, for the calibrated model and, to compare, for the nominal uncalibrated model `urdf/<urdf>.urdf`:
```bash
python3 generate_overlay.py --model panda_1 --data panda_1/front_left            # calibrated
python3 generate_overlay.py --model panda_1 --data panda_1/front_left --nominal  # nominal
```
The overlays are saved as `calibrated_urdf/<robot-name>/overlay_<tool-position-on-table>.png` and `overlay_nominal_<tool-position-on-table>.png`. By default the camera looks from behind the two sockets towards the robot and is zoomed such that all the configurations fit. To choose the view yourself, pass `--view window`, move the view in the window and press `q`; the view is saved and can be reused without the window with `--view file`. If you run the overlay again, pass `--no-generate-images` to reuse the images of the poses.

<table align="center">
  <tr>
    <td align="center" width="50%">
      <img src="imgs/overlay.PNG" width="400" alt="Overlay of the calibrated model" />
    </td>
    <td align="center" width="50%">
      <img src="imgs/overlaynominal.png" width="400" alt="Overlay of the nominal model" />
    </td>
  </tr>
  <tr>
    <td align="center"><b>Calibrated model</b><br />Consistent sphere position in each socket</td>
    <td align="center"><b>Nominal model</b><br />Scattered sphere positions</td>
  </tr>
</table>

If the optimization was successful, the forward kinematics of the calibrated model (left) is very consistent, as shown by the single red dot of the learned sphere attached at the end effector in each socket. For the nominal model (right), the forward kinematics is not consistent, as indicated by the scattered red dots.


## Print the tool
We used a BambuLab A1. We printed with classic PLA, with the finest available settings, i.e. 0.08 mm. The printer is very precise, when measuring the distance between the socket using a caliper, it will have an error of less than 0.1 mm. This is very desirable considering that we are using that to calibrate. The tool has two pairs of notches to fit calipers. Usually calipers have an angle of 45 deg or 30 deg, check on which side it fits the best. Use a caliper with a resolution of at least 0.01 mm. If you see that the distance is between 49.90 and 50.10 mm, it means that the print is perfectly calibrated. Otherwise, you can specify the distance you read in the optimization in meters. E.g.
```bash
python3 run_optimizer.py --model <nominal_urdf_name> --data <path/to/data/folder> --offset-distance 0.0501
```
We always found the printed tool to be at (almost) perfect distance of 50 mm!

<p align="center">
  <img src="imgs/caliper.jpeg" width="400" alt="Measuring the socket distance with a caliper" />
  <br />
  <b>Printed tool</b><br />Measuring the distance between the sockets with a caliper
</p>

The CAD file of the calibration tool available in the [`stl/MUKCa_tool.stl`](stl/MUKCa_tool.stl), which can also be copied and modified in [`Onshape`](https://cad.onshape.com/documents/0b6b115d8701048ec8c03433/w/dadf4b03313f5058948ebaa5/e/40893559aa99a64107618f54?renderMode=0&uiState=67e2e1d063975d2a0e09916f).


## Try it with your robot and your own urdf! 

### (Optional) Export the urdf from xacro
The calibration reads a plain urdf from `urdf/<arm_name>.urdf`. Set the description package, the `.urdf.xacro` path, its xacro arguments and the arm name at the top of [`export_urdf.launch.py`](ros2/calibration_tools/launch/export_urdf.launch.py), then:
```bash
ros2 launch calibration_tools export_urdf.launch.py
# -> urdf/<arm_name>.urdf, then use --model <arm_name>
```
See [`ros2/calibration_tools`](ros2/calibration_tools/README.md#export-a-urdf-from-xacro-optional) for details. Then add the sphere and the `config_optimizer` file as below.

### Add the sphere to the urdf
Add a link at the center of the sphere, fixed to the link that holds it (e.g. the flange). As in [`urdf/panda.urdf`](urdf/panda.urdf):
```xml
<link name="ball_link">
  <visual>
    <material name="red"><color rgba="1 0 0 1"/></material>
    <geometry><sphere radius="0.005"/></geometry>
  </visual>
</link>

<joint name="ball_joint" type="fixed">
  <origin rpy="0 0 0" xyz="0 0 0.04"/>
  <parent link="panda_hand_tcp"/>
  <child link="ball_link"/>
</joint>
```
- Name the link `ball_link`; the scripts look for this name.
- Add `ball_joint` to `config_optimizer/<model>.yaml`, so its offset is calibrated too.

### Tool dimensions
| Dimension | Where | Notes |
|---|---|---|
| Socket distance | `--offset-distance` of `run_optimizer.py` (m, default `0.05`) | Only needed if the caliper does not read 50 mm. |
| Flange to sphere center | `origin` of `ball_joint` in the urdf | Initial guess only, a few cm off is fine. It is calibrated. |
| Sphere radius | Not used | Only the sphere center matters. The `radius` only sets the size of the red dot in the overlay. |

### Example: Franka with a Bota sensor
There are situations where for example you want to change the end effector position in the URDF or where you want to put a sensor between the last joint and the hand. 
This is an example we have in the lab in TU Delft with the bota sensor. 

<p align="center">
  <img src="imgs/franka_with_bota_sensor.png" width="300" alt="Franka with a Bota force-torque sensor" />
  <br />
  <b>Franka with Bota sensor</b><br />Sensor mounted between the last joint and the hand
</p>

- I created a new urdf file in the homonym folder with the sensor located in between the two joints. I called the file `panda_plus_bota.urdf`
- I also added the mesh in the homonym folder so I can visualize it 
- I create a file `panda_plus_bota.yaml` in `config_optimizer` where I specify all the joints that are going to be optimized. Each joint has `xyz` `rpy` variables optimized as transformation from the previous joint. For example I choose to optimize all the first 7 joints, the ball transformation and also the transformation of the sensor since I do not know the perfect dimension of the sensor. 
  - `panda_joint1`
  - `panda_joint2`
  - `panda_joint3`
  - `panda_joint4`
  - `panda_joint5`
  - `panda_joint6`
  - `panda_joint7`
  - `bota_sensor_joint`
  - `ball_joint`
- Record the data. As far as you have a `joint_states` topic, you can run the command described at the beginning of these instructions. For example: 
```bash
ros2 run calibration_tools record_joint_states_dataset --joint-state-topic-name /joint_states --robot-name panda_with_sensor --tool-position-on-table front --joint-names panda_joint1 panda_joint2 panda_joint3 panda_joint4 panda_joint5 panda_joint6 panda_joint7
```
This will save the data in the directory `data` with the name `panda_with_sensor/front`. Nice. 

- Let's optimize the urdf now: 
```
python3 generate_complete_analysis.py --model panda_plus_bota --data panda_with_sensor/front
```
This will generate an optimized file in `calibrated_urdf` in the subfolder `panda_with_sensor` named `panda_plus_bota.urdf`. So now you can take your old broken urdf and swap it with the new one.


## Limitations 
The proposed method does not work for all the robots! Here is an example of performing the training on the kinova gen3lite robot. They do not have harmonic drives in each joint, hence they have more backlash. This is a problem, since we have uncertainty in the joint angles. Let's look at the bar plot before and after the training on different tool position. 

<p align="center">
  <img src="imgs/bar_plot_statistics_kinova.png" width="800" alt="Mean absolute error for different training and test positions of the Kinova Gen3 Lite" />
  <br />
  <b>Kinova Gen3 Lite</b><br />Mean absolute error before and after training on different tool positions
</p>

We can observe that training on the front brings the error down also on the left and on a position on the right (and higher) position of the tool. However, if we train only on the left, we observe overfitting, making the accuracy on the other position to decrease with respect to the original nominal model. For the kinova it is actually convenient to train on all the tool position. This brings the error down everywhere removing the influence of the noisy joint reading. 


## Cite the original work
```
Franzese, G., Spahn, M., Kober, J. and Della Santina, C., 2025. MUKCa: Accurate and Affordable Cobot Calibration Without External Measurement Devices. arXiv preprint arXiv:2503.12584.
```


## Troubleshooting

### Docker: no write access to `data` or `calibrated_urdf`
If the repository is mounted into a container that runs with a different uid than the owner of the files, the container cannot write recordings to `data` or results to `calibrated_urdf`. Give the container user write access on the host:
```bash
cd <path/to/kinematics_calibration>
setfacl -R -m u:<container uid>:rwX,d:u:<container uid>:rwX,d:u:<host uid>:rwX data calibrated_urdf
```
Use `id -u` in the container and on the host to get the uids. Undo it with `setfacl -R -b data calibrated_urdf`.
