# Calibrate Your Arm

Step-by-step kinematics calibration of `<your_arm>`, with several tool positions and a train / val split.

MUKCa records joint states only. The end-effector pose is implicit: the sphere is in socket 0 or socket 1 (50 mm apart).

## 1. Hardware
- Mount the sphere rigidly on `<your_arm>_tool0` (the flange link). Measure the flange → sphere center offset roughly.
- Fix the MUKCa tool to the table, in the `<your_arm>` workspace.

## 2. Start the robot (hand guiding)
Start `<your_arm>` in a mode where it can be moved by hand (e.g. gravity compensation only), so that it publishes `/joint_states`:
```bash
ros2 topic echo /joint_states --once   # check <your_arm>_joint_1..N
```

## 3. Config files
Copy the panda files and set the joints of `<your_arm>`:
| File | Usage |
|---|---|
| `ros2/calibration_tools/config/<your_arm>.yaml` | recorder (`robot_name: <your_arm>`, `joint_names: <your_arm>_joint_1..N`) |
| `config_optimizer/<your_arm>.yaml` | optimizer (`joints: <your_arm>_joint_1..N` and `ball_joint`) |

## 4. Record
```bash
source <workspace_dir>/install/setup.bash
ros2 launch calibration_tools record_joint_states_dataset.launch.py \
  config:=<your_path>/kinematics_calibration/ros2/calibration_tools/config/<your_arm>.yaml \
  tool_position_on_table:=train/front_vertical
# -> data/<your_arm>/train/front_vertical/hole_{0,1}.csv
```
Absolute path, or `config:=<your_arm>` after `colcon build --packages-select calibration_tools`.
`tool_position_on_table:=<set>/<position>` sets the dataset folder (layout in step 7). Use a new position name each time the tool moves (same name appends).

| Key | Action |
|---|---|
| `a` | add a point (sphere seated, arm still) |
| `d` | delete last point |
| `s` | switch socket |
| `q` | save and quit |

- ≥ 20 points per socket, with joint configurations as different as possible.
- Record several tool positions to cover the workspace; they are all trained together (step 6).

## 5. Prepare the model
Export the urdf of `<your_arm>` from its xacro:
```bash
ros2 launch calibration_tools export_urdf.launch.py \
  description_package:=<your_description_package> \
  xacro_file:=<path/to/your_arm.urdf.xacro> \
  xacro_args:="<your xacro args>" \
  arm_name:=<your_arm>
```
Use a single-arm xacro: the csv columns map to the urdf joints by position, so the urdf must have exactly the N recorded joints. Its joint names must match the recorded names (`<your_arm>_joint_1..N`).

Add the sphere to `urdf/<your_arm>.urdf` (offset from step 1):
```xml
<link name="ball_link"/>
<joint name="ball_joint" type="fixed">
  <origin rpy="0 0 0" xyz="0 0 0.04"/>
  <parent link="<your_arm>_tool0"/>
  <child link="ball_link"/>
</joint>
```
Check:
```bash
grep -c 'type="revolute"' urdf/<your_arm>.urdf   # N
```

## 6. Calibrate
```bash
cd kinematics_calibration/scripts
R=<your_arm>; P="front_tilted front_vertical right_vertical"   # your tool positions
T=""; V=""; for p in $P; do T="$T $R/train/$p"; V="$V $R/val/$p"; done
python3 run_optimizer.py --model <your_arm> --root-link <your_arm>_base_link --data $T
python3 generate_learning_curve.py --model $R/train --test $V
python3 compute_improved_performance.py --model $R/train --test $V
python3 plot_mae_sigma.py --model $R/train              # -> mae_vs_sigma_<position>_train.png
python3 plot_mae_sigma.py --model $R/train --data $V    # -> mae_vs_sigma_<position>_val.png
```
Result: `calibrated_urdf/<your_arm>/train/<your_arm>.urdf` (output folder = parent of the first `--data`).

`plot_mae_sigma.py` → `mae_vs_sigma_<position>_<set>.png`, nominal vs calibrated:
- like the README scheme, one row per model (nominal, calibrated).
- each socket: predicted sphere positions (dots) around their centre `+`, on true mm axes (same scale for both sockets of a row); circle and arrow = σ, MAE in the title.
- between the sockets: `d` = predicted socket distance, target 50 mm, offset error ε (the gap itself is not to scale, `//`).
- table: σ and MAE per socket, `d` and ε, nominal vs calibrated.

Metrics (per socket, from the FK sphere positions of all poses):
- σ (consistency) = RMS distance to the socket centre; MAE = mean distance (σ ≥ MAE).
- ε (distortion) = |distance between the two socket centres − 50 mm|.

## 7. Data layout (train / val)
```
data/<your_arm>/
  train/<position>/hole_{0,1}.csv   # fitted together, all positions
  val/<position>/hole_{0,1}.csv     # held-out points, same tool position
```
- `<set>/<position>`, not `<position>/<set>`: the scripts name each dataset by its folder name (must be unique, i.e. the position), and the output goes to `calibrated_urdf/<parent of the first --data>`, one model for all positions.
- Val: record it directly (`tool_position_on_table:=val/<position>`, tool not moved), or move ~5 random rows per socket from `train/` to `val/`.
- Each position counts equally in the optimizer, whatever its number of points; each has its own socket centres.
- Without `--test`, the eval scripts use the other folders of `data/<model>` as test sets.
- Extra experiments: `run_optimizer.py --output <your_arm>/<name>` writes to `calibrated_urdf/<your_arm>/<name>/` instead of overwriting `train/`. Example: `holdout_<position>/`, trained without one position and tested on it. `plot_mae_sigma.py --set test` labels the plot of such an unseen position.
