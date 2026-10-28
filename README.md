# MUKCa: Minimal User-friendly Kinematics Calibration of Robot Manipulators

Have you ever wondered why the accuracy of your robot is poor, and why the end-effector is at different poses when you move in the null-space? Well, the platonics did! And we are here to help you out. This problem is especially relevant when working with cartesian control where an accurate forward kinematics is desirable.

The method implemented in this repository was mainly developed for the Franka Emika Panda robot, where inaccurate kinematics have been raised a few times but was tested on many other commercial robots. This is the first tool that does not require any external systems like optitrack, camera sensors or laser trackers. You will only need one simple tool to place in front of the robot. The tool is 3D printable and it is composed of two spherical sockets placed at a distance of 50 mm. A 3D printable spherical joint is then attached to the end effector and used to record many positions of the robot in each of the sockets, identified as 0 and 1.
![alt text](imgs/mukca.png)
The first step is the data recording and then the optimization is performed to optimize the robot urdf file. 


## What do we propose in this new approach? 
<!-- ![Alt Text](imgs/Calibration_scheme.png) -->

The method contribution is the Minimalist and User-friendly Kinematics Calibration (MUKCa) available in the [stl/MUKCa_tool.stl](stl/MUKCa_tool.stl) folder, as the combination of an affordable calibration tool, illustrated in figure and an optimization algorithm for kinematic parameter identification. The proposed minimalist tool is designed for accessibility and affordability, and it is 3D printable and composed of a sphere (or ball) with a two-socket base, as illustrated, without relying on external measurement systems or camera. After printing them, the calibration can immediately start with the data recording on the robot by simply placing the tool in front of the robot and the sphere attached to the end-effector.

The two sockets are printed at a known distance, and the ball is printed to have the minimal possible play in the sockets. 

We propose a novel optimization routine. After collecting set of joint configurations for each socket, the calibration routine minimizes the variance of the predicted ball position for each of the two sockets, maximizing the model nullspace consistency.  Moreover, the model is optimized such that the distance among the average prediction matches the known distance between the two sockets, minimizing the model volumetric distortion. This consistency-and-distortion optimization is the first of its kind in the field of kinematic calibration. 


## Overview
The calibration has two independent steps that share only the recorded csv files:

| Step | What | Environment |
|---|---|---|
| 1. Record | ROS 2 package [ros2/calibration_tools](ros2/calibration_tools) saves the joint configurations with the ball in socket 0 and socket 1 to `data/<robot-name>/<tool-position-on-table>/hole_{0,1}.csv` | ROS 2 Humble |
| 2. Calibrate | Python package `calibrate_fk` and the `scripts` folder optimize the URDF on the recorded data and write it to `calibrated_urdf/<robot-name>/` | Python virtual environment, no ROS |


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

Check the build and run the unit tests:
```bash
ros2 pkg executables calibration_tools   # -> calibration_tools record_joint_states_dataset
colcon test --packages-select calibration_tools && colcon test-result --verbose
```

With `--symlink-install`, changes to the Python files take effect without rebuilding. In a new terminal, only `. <workspace_dir>/install/setup.bash` is needed.


### Calibration (Python, no ROS)
Install the python package `calibrate_fk` in a virtual environment, so that pip does not replace the numpy, scipy and matplotlib used by ROS:
```bash
python3 -m virtualenv ~/.venvs/mukca   # or: python3 -m venv ~/.venvs/mukca, or: uv venv -p 3.10 ~/.venvs/mukca
~/.venvs/mukca/bin/pip install -e <path/to/kinematics_calibration>
```
Or install it through poetry with `poetry install`. All dependencies are available for x86_64 and aarch64 (e.g. Jetson with JetPack 6), Python 3.10. Create the environment on each machine, do not copy it.


### Docker
If the repository is mounted into a container that runs with a different uid than the owner of the files, the container cannot write recordings to `data` or results to `calibrated_urdf`. Give the container user write access on the host:
```bash
cd <path/to/kinematics_calibration>
setfacl -R -m u:<container uid>:rwX,d:u:<container uid>:rwX,d:u:<host uid>:rwX data calibrated_urdf
```
Use `id -u` in the container and on the host to get the uids. Undo it with `setfacl -R -b data calibrated_urdf`.


## Quick start without a robot
The calibration runs on the datasets in the `data` folder:
```bash
source ~/.venvs/mukca/bin/activate
cd <path/to/kinematics_calibration>/scripts
export MPLBACKEND=Agg   # save the plots to files instead of opening windows
python3 run_optimizer.py --model panda --data panda_1/front_center
python3 generate_learning_curve.py --model panda_1
python3 compute_improved_performance.py --model panda_1
```
This removes about 97 % of the consistency error on the training data and about 95 % on the 7 tool positions of `panda_1` that were not used for training. See [Calibrate the urdf model](#calibrate-the-urdf-model) for what each script does.

To try the recorder without a robot, publish a fake joint state in one terminal:
```bash
ros2 topic pub -r 20 /joint_states sensor_msgs/msg/JointState \
  "{name: [panda_joint1, panda_joint2, panda_joint3, panda_joint4, panda_joint5, panda_joint6, panda_joint7],
    position: [0.0, -0.5, 0.0, -2.0, 0.0, 1.6, 0.8]}"
```
and run the recorder as described below with `--robot-name panda_test` in a second terminal. Delete `data/panda_test` afterwards.


## Record the data
The tool has two sockets identified as 0 and 1. There is a sphere attached at the end effector. The idea is to record many joint configurations with the ball in socket 0 and many others with the ball in socket 1, at least 20 for each socket.

1. Start the driver or controller of your robot, and check that it publishes a [sensor_msgs/msg/JointState](https://docs.ros.org/en/humble/p/sensor_msgs/interfaces/msg/JointState.html) topic:
   ```bash
   ros2 topic echo /joint_states --once
   ```
2. Run the recorder, e.g. for a KUKA iiwa14:
   ```bash
   ros2 run calibration_tools record_joint_states_dataset \
     --joint-state-topic-name /joint_states \
     --robot-name kuka_1 \
     --tool-position-on-table front \
     --joint-names iiwa_joint_1 iiwa_joint_2 iiwa_joint_3 iiwa_joint_4 iiwa_joint_5 iiwa_joint_6 iiwa_joint_7
   ```
3. Type the keys in the terminal running the recorder (no Enter needed):
   - Press 'a' to add a new joint configuration with the sphere in the socket that is displayed in green.
   - Press 'd' to delete the last joint configuration from the list.
   - Press 's' when you switch the socket in which you are recording.
   - Press 'i' to show the help.
   - Press 'q' when you have finished recording, to save the data and quit.
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

See [ros2/calibration_tools/README.md](ros2/calibration_tools/README.md) for all options, e.g. `--data-dir` to save the data somewhere else.


### Record data with a Franka
![Alt Text](imgs/mukca_franka.gif)

With ROS 2, run `record_joint_states_dataset` on the `/joint_states` of your Franka controller, with the `panda_joint1` ... `panda_joint7` joint names. It is important to check that the joint state can be echoed.

The ROS 1 recorder `record_joint_states_panda` in `ros_ws`, that uses the buttons on the end effector to add data or switch between holes, together with the [Human Friendly Controllers](https://github.com/franzesegiovanni/franka_human_friendly_controllers.git), is not ported to ROS 2.


## Calibrate the urdf model
Activate the virtual environment and go to the **scripts folder**:
```bash
source ~/.venvs/mukca/bin/activate
cd <path/to/kinematics_calibration>/scripts
export MPLBACKEND=Agg   # optional: save the plots to files instead of opening windows
```

Three scripts are run in sequence. Only the first one changes the model; the other two evaluate its result and can be run in any order afterwards. `generate_complete_analysis.py` runs all of them, including the overlay below.

| # | Script | What it does | Output |
|---|---|---|---|
| 1 | `run_optimizer.py` | Optimizes the xyz rpy of the joints listed in `config_optimizer/<model>.yaml`, such that the predicted ball position is consistent in each socket and the two sockets are 50 mm apart | `calibrated_urdf/<robot-name>/<model>.urdf`, `config.yaml` and the model after each optimization step `step_*/model.urdf` |
| 2 | `generate_learning_curve.py` | Evaluates every optimization step on the training data and on all the other datasets in `data/<robot-name>` | `consistency.png`, `distortion.png` |
| 3 | `compute_improved_performance.py` | Compares the nominal and the calibrated model on the training data and on the other datasets, and prints the parameter changes | Terminal output |


### 1. Optimize
```bash
python3 run_optimizer.py --model <nominal_urdf_name> --data <robot-name>/<tool-position-on-table>
```
The nominal model needs to be in the **urdf** folder, e.g. `panda` for `urdf/panda.urdf`. The data are the ones saved in the previous step, e.g. `panda_1/front`. The script prints the error before and after the optimization, and writes a new urdf with the same name as the original one to the **calibrated_urdf** folder, in the subfolder of the robot name. So `calibrated_urdf/panda_1/panda.urdf` contains the calibrated parameters. Running the script again overwrites this folder.
```bash
python3 run_optimizer.py --model panda --data panda_1/front
```
If the printed tool is not exactly 50 mm, pass the distance measured with a caliper in meters with `--offset-distance`, see [Print the tool](#print-the-tool).

Training on one position is sufficient, but training on multiple tool positions that are placed in different parts of the workspace is as easy as specifying their paths in the --data input. All the paths must belong to the same robot:
```bash
python3 run_optimizer.py --model panda --data panda_1/front panda_1/left panda_1/right panda_1/high
```
The optimization will now try to maximize the consistency in any of the sockets of each tool position and minimize the bias in the predicted distance between the two sockets.
![alt text](imgs/bar_plot_panda_2.jpg)
In this plot we trained and evaluated on different positions of the tool or in all of them. You can see that even when training only on the front, the final accuracy on the right-high position is still good. By training on all the positions, the error goes down everywhere.


### 2. Plot the learning curves
```bash
python3 generate_learning_curve.py --model panda_1
```
This plots the consistency and the distortion of the model both on the training set and on any other dataset in the folder `data/panda_1`, and saves them in `calibrated_urdf/panda_1`.
For example:
<p float="left">
  <img src="imgs/consistency.png" alt="Consistency" width="300" />
  <img src="imgs/distortion.png" alt="Distortion" width="300" />
</p>
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
You can generate also an overlay of the robot in the different configurations that were recorded. For example 
```bash
python3 generate_overlay.py  --model panda_1 --data panda_1/front 
```
This will prompt a window with a visualization of the robot. Move the view such that you can nicely see the end-effector. Press q in the visualization window to save the camera setting with the point of view.
If you run the overlay again, you can also avoid to generate all the images of the poses. Just pass the flag --no-generate-images .

If the optimization was successful the overlay will look like this: 


![Alt Text](imgs/overlay.PNG)

You can notice that the forward kinematics is very consistent by the displayed red dot of the learned sphere attached at the end effector. 

The overlay of the nominal uncalibrated model, will look more like this:

![Alt Text](imgs/overlaynominal.png)

In this case, you can see that the forward kinematics is not consistent, as indicated by the scattered red dots of the learned sphere attached at the end effector.


### Use the panda urdf for a calibrated Cartesian impedance controller. 

You can use the [Franka human friendly controllers](https://github.com/franzesegiovanni/franka_human_friendly_controllers) and place the calibrated model in the urdf folder. To be sure to generate a compatible urdf model that will have the right paths specified, please run: 

```bash
python3 convert_panda_urdf.py -m panda_1
```
this will generate a **panda_calibrated.urdf** in the panda_1 folder. Copy this file in the controller repo ( in the urdf directory) and follow the instructions on how to start the controller using the calibrated external model. 


# Print the tool
We used a BambuLab A1. We printed with classic PLA, with the finest available settings, i.e. 0.08 mm. The printer is very precise, when measuring the distance between the socket using a caliper, it will have an error of less than 0.1 mm. This is very desirable considering that we are using that to calibrate. The tool has two pairs of notches to fit calipers. Usually calipers have an angle of 45 deg or 30 deg, check on which side it fits the best. Use a caliper with a resolution of at least 0.01 mm. If you see that the distance is between 49.90 and 50.10 mm, it means that the print is perfectly calibrated. Otherwise, you can specify the distance you read in the optimization in meters. E.g.
```bash
python3 run_optimizer.py --model <nominal_urdf_name> --data <path/to/data/folder> --offset-distance 0.0501
```
We always found the printed tool to be at (almost) perfect distance of 50 mm!
![Alt Text](imgs/caliper.jpeg)

You can remix the tools, I shared them in an [Onshape](https://cad.onshape.com/documents/0b6b115d8701048ec8c03433/w/dadf4b03313f5058948ebaa5/e/40893559aa99a64107618f54?renderMode=0&uiState=67e2e1d063975d2a0e09916f).


# Try it with your robot and your own urdf! 
There are situations where for example you want to change the end effector position in the URDF or where you want to put a sensor between the last joint and the hand. 
This is an example we have in the lab in TU Delft with the bota sensor. 
![Alt Text](imgs/franka_with_bota_sensor.png)
- I created a new urdf file in the homonym folder with the sensor located in between the two joints. I called the file panda_plus_bota.urdf
- I also added the mesh in the homonym folder so I can visualize it 
- I create a file panda_plus_bota.yaml in config_optimizer where I specify all the joints that are going to be optimized. Each joint has xyz rpy variable optimized as transformation from the previous joint. For example I choose to optimize all the first 7 joints, the ball transformation and also the transformation of the sensor since I do not know the perfect dimension of the sensor. 
  - panda_joint1
  - panda_joint2
  - panda_joint3
  - panda_joint4
  - panda_joint5
  - panda_joint6
  - panda_joint7
  - bota_sensor_joint
  - ball_joint
- Record the data. As far as you have a joint_states topic, you can run the command described at the beginning of these instructions. For example: 
```bash
ros2 run calibration_tools record_joint_states_dataset --joint-state-topic-name /joint_states --robot-name panda_with_sensor --tool-position-on-table front --joint-names panda_joint1 panda_joint2 panda_joint3 panda_joint4 panda_joint5 panda_joint6 panda_joint7
```
This will save the data in the directory `data` with the name `panda_with_sensor/front`. Nice. 

- Let's optimize the urdf now: 
```
python3 generate_complete_analysis.py --model panda_plus_bota --data panda_with_sensor/front
```
This will generate an optimized file in `calibrated_urdf` in the subfolder `panda_with_sensor` named `panda_plus_bota.urdf`. So now you can take your old broken urdf and swap it with the new one.


<!-- # Calibrate a bimanual setup (Experimental ⚠️)
> **Note:** `scripts/bimanual_calibration.py` was removed upstream in commit `e393c09`; restore its last version with `git show e393c09^:scripts/bimanual_calibration.py > scripts/bimanual_calibration.py` if needed. The recording commands below use the ROS 1 Franka recorder in `ros_ws`.

Place the tool in front of the two robots. We are going to use the tool now to identify the chosen global frame. Place the tool in the y direction with the hole 1 in the direction of the positive y. Place it also on a flat table that is perpendicular to the gravity vector. I measure left and right from the perspective of the user looking at the robots. 

You calibrate each of the robots separately before. This last stage is not to calibrate the kinematics but only the relative position of the base. 
You will have to record another dataset for each robot for each socket. 
For example: 
```bash
rosrun calibration_tools record_joint_states_panda --joint-state-topic-name /joint_states --robot-name panda_left --config-file panda_left.yaml --tool-position-on-table world_frame 
```
and for the robot on the right

```bash
rosrun calibration_tools record_joint_states_panda --joint-state-topic-name /joint_states --robot-name panda_right --config-file panda_right.yaml --tool-position-on-table world_frame 
```
Perfect, now we compute the relative position of the arms in the world frames that we just placed in the current location of the MUKCa tool: 

```
python3 bimanual_calibration.py --robot_left panda_2  --robot_right panda_3 --data_left panda_left/world_frame  --data_right panda_right/world_frame
```

This script will save a file called relative_position.yaml folder of the workspace. 
⚠️ Warning!
The bimanual calibration is only experimental and was only tested on a limited case. Moreover, currently the authors believe that the bimanual calibration made in this way is an ill-posed problem. This is not part of the official MUKCa calibration procedure.  -->


# Limitations 
The proposed method does not work for all the robots! Here is an example of performing the training on the kinova gen3lite robot. They do not have harmonic drives in each joint, hence they have more backlash. This is a problem, since we have uncertainty in the joint angles. Let's look at the bar plot before and after the training on different tool position. 
![alt text](imgs/bar_plot_statistics_kinova.png)
We can observe that training on the front brings the error down also on the left and on a position on the right (and higher) position of the tool. However, if we train only on the left, we observe overfitting, making the accuracy on the other position to decrease with respect to the original nominal model. For the kinova it is actually convenient to train on all the tool position. This brings the error down everywhere removing the influence of the noisy joint reading. 


# Cite this work.
This new method is under review for publication.
You can cite it the preprint:
```
Franzese, G., Spahn, M., Kober, J. and Della Santina, C., 2025. MUKCa: Accurate and Affordable Cobot Calibration Without External Measurement Devices. arXiv preprint arXiv:2503.12584.
```
If you calibrated your URDF and used it in your research, and found it helpful, please reach out to Giovanni (g.franzese@tudelft.nl). If you are in a company and calibrate your robot using the MUKCa tool, please acknowledge it.


# Give feedback!

Any feedback on the usability, the method, and the results is appreciated and welcome! Let's make collaborative robots accurate together.
