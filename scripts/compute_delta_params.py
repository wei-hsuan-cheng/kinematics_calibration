"""Write urdf_delta_params.yaml: calibrated urdf parameters compared with the nominal urdf.

For each optimized joint (config_optimizer/<model>.yaml), with the change expressed in the
joint's own (nominal) frame:
  dxyz_m, drpy_rad  : for config. Calibrated origin = nominal xyz + dxyz_m, nominal rpy + drpy_rad
  dxyz_mm, drpy_deg : the same, in mm / deg, to read
  rot_deg           : small rotation R_nominal^T * R_calibrated as a rotation vector [deg], to read
                      revolute joint with axis z: rx, ry = axis tilt, rz = zero (encoder) offset

Usage:
  python3 compute_delta_params.py --model <your_arm>/train
  -> calibrated_urdf/<your_arm>/train/urdf_delta_params.yaml (+ markdown table on stdout)
"""
import argparse
import glob
import os
import xml.etree.ElementTree as ET

import numpy as np
import yaml

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir))


def rpy_to_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def rotation_vector(R):
    theta = np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))
    if theta < 1e-12:
        return np.zeros(3)
    return theta / (2 * np.sin(theta)) * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])


def load_joints(urdf_file):
    """Kinematic joints only (direct children of <robot>, not the ros2_control <joint> tags)."""
    joints = {}
    for joint in ET.parse(urdf_file).getroot().findall('joint'):
        origin = joint.find('origin')
        axis = joint.find('axis')
        xyz = [float(v) for v in (origin.get('xyz', '0 0 0') if origin is not None else '0 0 0').split()]
        rpy = [float(v) for v in (origin.get('rpy', '0 0 0') if origin is not None else '0 0 0').split()]
        joints[joint.get('name')] = {
            'type': joint.get('type'),
            'axis': axis.get('xyz') if axis is not None else None,
            'xyz': np.array(xyz),
            'rpy': rpy,
        }
    return joints


def r(values, digits):
    return [round(float(v), digits) for v in values]


def compute_delta_params(nominal_urdf, calibrated_urdf, joint_names):
    nominal, calibrated = load_joints(nominal_urdf), load_joints(calibrated_urdf)
    deltas = {}
    for name in joint_names:
        n, c = nominal[name], calibrated[name]
        dxyz_m = c['xyz'] - n['xyz']
        # plain rpy difference, wrapped to (-pi, pi]: nominal rpy + drpy = calibrated rotation
        drpy_rad = np.arctan2(np.sin(np.subtract(c['rpy'], n['rpy'])), np.cos(np.subtract(c['rpy'], n['rpy'])))
        rot = np.degrees(rotation_vector(rpy_to_matrix(*n['rpy']).T @ rpy_to_matrix(*c['rpy'])))
        deltas[name] = {
            'type': n['type'],
            'axis': n['axis'],
            # human readable
            'dxyz_mm': r(dxyz_m * 1000, 3),
            'dpos_norm_mm': round(float(np.linalg.norm(dxyz_m) * 1000), 3),
            'drpy_deg': r(np.degrees(drpy_rad), 4),
            'rot_deg': r(rot, 4),
            'drot_norm_deg': round(float(np.linalg.norm(rot)), 4),
            # for config: origin = nominal + delta
            'dxyz_m': r(dxyz_m, 12),
            'drpy_rad': r(drpy_rad, 12),
            'nominal': {'xyz': r(n['xyz'], 9), 'rpy': r(n['rpy'], 9)},
            'calibrated': {'xyz': r(c['xyz'], 12), 'rpy': r(c['rpy'], 12)},
        }
    return deltas


def markdown_table(deltas):
    lines = ["| Joint | Δ position (mm) [x, y, z] | \\|Δp\\| mm | axis tilt rx, ry (°) | zero offset rz (°) | \\|Δθ\\| ° |",
             "|---|---|---|---|---|---|"]
    for name, d in deltas.items():
        dx = ", ".join(f"{v:+.2f}" for v in d['dxyz_mm'])
        if d['type'] == 'fixed':
            tilt, zero = "–", "–"
        else:
            tilt = f"{d['rot_deg'][0]:+.3f}, {d['rot_deg'][1]:+.3f}"
            zero = f"{d['rot_deg'][2]:+.3f}"
        lines.append(f"| `{name}` | {dx} | {d['dpos_norm_mm']:.2f} | {tilt} | {zero} | {d['drot_norm_deg']:.3f} |")
    return "\n".join(lines)


HEADER = """\
# Kinematics calibration: change of each joint origin, calibrated vs nominal urdf.
# Written by kinematics_calibration/scripts/compute_delta_params.py.
#
# For config (robot description): calibrated origin = nominal + delta, exactly
#   dxyz_m   : calibrated xyz - nominal xyz [m], parent frame
#   drpy_rad : calibrated rpy - nominal rpy [rad], component-wise, wrapped to (-pi, pi]
#              urdf rpy = [roll, pitch, yaw], R = Rz(yaw) Ry(pitch) Rx(roll)
#              A difference of the rpy numbers, not a rotation: only valid on the same
#              nominal rpy (see nominal below), and not to be read physically.
#
# To read (human):
#   dxyz_mm, dpos_norm_mm : dxyz_m in mm, and its norm
#   drpy_deg              : drpy_rad in deg (which urdf angle numbers changed)
#   rot_deg, drot_norm_deg: the physical rotation R_nominal^T R_calibrated as a rotation
#                           vector [rx, ry, rz] in the nominal joint frame [deg], and its angle.
#                           Revolute joint with axis z: rx, ry = axis tilt, rz = zero (encoder) offset.
#   Example, a joint with nominal rpy -90, 0, 90 deg: drpy_deg [0, 0.27, 0] and rot_deg [0, 0, 0.27]
#   = the pitch number changed, which for this joint is a 0.27 deg rotation about the joint axis,
#   i.e. an encoder zero offset.
#   Near pitch +-90 deg roll and yaw trade off in drpy_deg (e.g. -0.28 / +0.28): read rot_deg.
#
# nominal, calibrated: the joint origins of both urdfs (calibrated = nominal + dxyz_m, drpy_rad).
"""


def write_delta_params(model_folder, nominal_urdf, joint_names, output_name="urdf_delta_params.yaml"):
    calibrated_urdf = os.path.join(model_folder, os.path.basename(nominal_urdf))
    deltas = compute_delta_params(nominal_urdf, calibrated_urdf, joint_names)
    content = {
        'nominal_urdf': os.path.relpath(nominal_urdf, ROOT),
        'calibrated_urdf': os.path.relpath(calibrated_urdf, ROOT),
        'joints': deltas,
    }
    out = os.path.join(model_folder, output_name)
    with open(out, "w") as f:
        f.write(HEADER)
        yaml.dump(content, f, sort_keys=False, default_flow_style=None)
    return out, deltas


def main():
    parser = argparse.ArgumentParser(description="Write urdf_delta_params.yaml (calibrated vs nominal urdf parameters)")
    parser.add_argument("--model", "-m", required=True, help="Calibrated model folder in calibrated_urdf, e.g. <your_arm>/train")
    parser.add_argument("--urdf", "-u", default=None, help="Nominal urdf name in urdf/ (default: the .urdf in the model folder)")
    args = parser.parse_args()

    model_folder = os.path.join(ROOT, 'calibrated_urdf', args.model)
    if args.urdf is None:
        urdfs = glob.glob(os.path.join(model_folder, "*.urdf"))
        if len(urdfs) != 1:
            raise SystemExit(f"Expected one .urdf in {model_folder}, found {len(urdfs)}: pass --urdf")
        urdf_name = os.path.splitext(os.path.basename(urdfs[0]))[0]
    else:
        urdf_name = args.urdf
    nominal_urdf = os.path.join(ROOT, 'urdf', urdf_name + ".urdf")
    with open(os.path.join(ROOT, 'config_optimizer', urdf_name + ".yaml")) as f:
        joint_names = yaml.safe_load(f)['joints']

    out, deltas = write_delta_params(model_folder, nominal_urdf, joint_names)
    print(markdown_table(deltas))
    print(f"\n-> {out}")


if __name__ == "__main__":
    main()
