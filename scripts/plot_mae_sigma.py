"""Plot the socket spread and the socket distance, nominal vs calibrated model, like the README scheme.

For each recorded pose the forward kinematics predicts the sphere centre p_i. The socket centre c_k
is the mean of the predictions in socket k (it is never measured):
    d_i   = |p_i - c_k|
    MAE   = mean(d_i)                      (mean_absolute_error)
    sigma = sqrt(mean(d_i^2))              (std_dev_fk, the spread the optimizer reduces)
    d     = |c_0 - c_1|                    (predicted socket distance, target `offset` = 50 mm)
    eps   = |d - offset|                   (distance_error)

One row per model (nominal, calibrated), then a table of the numbers. Each socket shows its predicted sphere positions around the
centre (+) on true mm axes (same scale for both sockets of a row) with a circle and arrow of
radius sigma; the arrow between the sockets is d (the gap itself is not to scale, marked //).

Usage:
    python3 plot_mae_sigma.py --model <your_arm>                        # training data of the model
    python3 plot_mae_sigma.py --model <your_arm> --data <your_arm>/front
    python3 plot_mae_sigma.py --model <your_arm>/train --data <your_arm>/val/front_vertical
Saves calibrated_urdf/<model>/mae_vs_sigma_<tool-position>[_<set>].png,
<set> = train / val / test when the data is in such a split folder.
"""

import argparse
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402
import yourdfpy  # noqa: E402
from matplotlib.patches import Circle  # noqa: E402

from calibrate_fk.utils import evaluate_model  # noqa: E402

SURFACE, INK, INK2, MUTED = "#fcfcfb", "#0b0b0b", "#52514e", "#8a8984"
SOCKET_COLORS = ("#2a78d6", "#eb6834")


def socket_clouds(model_file, data_folder, offset_mm):
    """Return {position: clouds} in mm: each socket's points around its own centre."""
    model = yourdfpy.URDF.load(model_file, load_meshes=False, build_scene_graph=True)
    result = {}
    for position, stats in evaluate_model(model, [data_folder]).items():
        points = [np.array(stats["fks_1"]) * 1e3, np.array(stats["fks_2"]) * 1e3]
        centres = [p.mean(axis=0) for p in points]
        deviations = [p - c for p, c in zip(points, centres)]
        dists = [np.linalg.norm(d, axis=1) for d in deviations]
        distance = np.linalg.norm(centres[1] - centres[0])
        # view: x along the socket axis, y the main spread perpendicular to it
        u = (centres[1] - centres[0]) / distance
        perpendicular = np.vstack([d - np.outer(d @ u, u) for d in deviations])
        w = np.linalg.svd(perpendicular, full_matrices=False)[2][0]
        result[position] = {
            "xy": [np.column_stack([d @ u, d @ w]) for d in deviations],
            "mae": [d.mean() for d in dists],
            "sigma": [np.sqrt((d ** 2).mean()) for d in dists],
            "distance": distance,
            "eps": distance - offset_mm,
        }
    return result


def style_socket_axes(ax, lim):
    ax.set_facecolor(SURFACE)
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_aspect("equal")
    ax.grid(color="#e4e3df", lw=0.8, zorder=0)
    ax.axhline(0, color="#d2d1cc", lw=0.9, zorder=0)
    ax.axvline(0, color="#d2d1cc", lw=0.9, zorder=0)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(MUTED)
    ax.tick_params(colors=INK2, labelsize=8.5, length=3)
    ax.set_xlabel("along socket axis [mm]", fontsize=9, color=INK2)
    ax.set_ylabel("perpendicular [mm]", fontsize=9, color=INK2)


def draw_row(fig, cell, title, clouds, offset_mm):
    """One model: socket 0 | d | socket 1, both sockets on true, identical mm axes."""
    sub = cell.subgridspec(1, 3, width_ratios=[1, 0.72, 1], wspace=0.04)
    ax0, ax_mid, ax1 = (fig.add_subplot(sub[0, i]) for i in range(3))
    lim = max(max(np.abs(xy).max() for xy in clouds["xy"]), max(clouds["sigma"])) * 1.3
    arrow_angle = (np.deg2rad(135), np.deg2rad(45))   # like the scheme: up-left, up-right

    for s, ax in ((0, ax0), (1, ax1)):
        style_socket_axes(ax, lim)
        color = SOCKET_COLORS[s]
        sigma, mae = clouds["sigma"][s], clouds["mae"][s]
        ax.add_patch(Circle((0, 0), sigma, facecolor=color, alpha=0.08, edgecolor="none", zorder=1))
        ax.add_patch(Circle((0, 0), sigma, fill=False, edgecolor="#1f2937", lw=2.6, zorder=2))
        xy = clouds["xy"][s]
        ax.scatter(xy[:, 0], xy[:, 1], s=110, color=color, edgecolor=SURFACE, linewidth=1.6, zorder=3)
        ax.plot(0, 0, marker="+", color="#1f2937", ms=22, mew=3, zorder=5)
        a = arrow_angle[s]
        tip = (sigma * np.cos(a), sigma * np.sin(a))
        ax.annotate("", xy=tip, xytext=(0, 0), zorder=6,
                    arrowprops={"arrowstyle": "-|>", "color": color, "lw": 2.6, "mutation_scale": 16,
                                "shrinkA": 0, "shrinkB": 0})
        ax.text(tip[0], tip[1] + lim * 0.05, f"σ = {sigma:.2f} mm",
                ha="center", va="bottom", fontsize=11, color=color,
                fontweight="bold", zorder=7,
                bbox={"facecolor": SURFACE, "edgecolor": "none", "alpha": 0.85, "pad": 1.5})
        ax.set_title(f"socket {s}    MAE {mae:.2f} mm", fontsize=11, color=color, fontweight="bold")
        if s == 1:
            ax.set_ylabel("")   # same axes as socket 0

    # middle: d between the two centres (the gap is not to scale, marked //)
    ax_mid.set_axis_off()
    ax_mid.set_xlim(0, 1)
    ax_mid.set_ylim(-1, 1)
    ax_mid.annotate("", xy=(0.9, 0), xytext=(0.06, 0),
                    arrowprops={"arrowstyle": "<|-|>", "color": INK2, "lw": 1.4, "mutation_scale": 12})
    for dx in (-0.025, 0.025):
        ax_mid.plot([0.5 + dx - 0.02, 0.5 + dx + 0.02], [-0.06, 0.06], color=INK2, lw=1.4)
    ax_mid.text(0.5, 0.1, f"d = {clouds['distance']:.3f} mm", ha="center", va="bottom", fontsize=12,
                color=INK, fontweight="bold")
    ax_mid.text(0.5, -0.1, f"target {offset_mm:.0f} mm\noffset error ε = {clouds['eps']:+.3f} mm",
                ha="center", va="top", fontsize=10, color=INK2, linespacing=1.4)
    ax_mid.text(0.5, -0.55, "centre to centre\n(gap not to scale)", ha="center", va="top",
                fontsize=8.5, color=MUTED, linespacing=1.3)

    y_title = ax0.get_position().y1 + 0.03
    fig.text(0.03, y_title, title, fontsize=14, fontweight="bold", color=INK)


def draw_table(ax, nominal, calibrated):
    """Simple table: metric | nominal | calibrated | change."""
    ax.set_axis_off()
    rows = []
    for s in (0, 1):
        rows.append((f"socket {s}  σ", nominal["sigma"][s], calibrated["sigma"][s]))
        rows.append((f"socket {s}  MAE", nominal["mae"][s], calibrated["mae"][s]))
    rows.append(("socket distance d  (target 50)", nominal["distance"], calibrated["distance"]))
    rows.append(("offset error |ε|", abs(nominal["eps"]), abs(calibrated["eps"])))

    cells = []
    for label, before, after in rows:
        pct = (after / before - 1) * 100
        change = "" if label.startswith("socket distance") else f"{'−' if pct < 0 else '+'}{abs(pct):.1f} %"
        cells.append([label, f"{before:.3f} mm", f"{after:.3f} mm", change])
    table = ax.table(cellText=cells, colLabels=["", "nominal", "calibrated", "change"],
                     colWidths=[0.34, 0.18, 0.18, 0.14], loc="center", cellLoc="right")
    table.auto_set_font_size(False)
    table.set_fontsize(10)
    table.scale(1, 1.45)
    for (row, col), cell in table.get_celld().items():
        cell.set_edgecolor("#e4e3df")
        cell.set_linewidth(0.8)
        cell.set_facecolor(SURFACE if row % 2 else "#f3f2ee")
        if col == 0:
            cell.set_text_props(ha="left", color=INK)
            cell.PAD = 0.03
        if row == 0:
            cell.set_text_props(color=INK2, fontweight="bold")
            cell.set_facecolor("#ebeae5")
        elif col == 2:
            cell.set_text_props(fontweight="bold", color=INK)
        elif col == 3:
            worse = cell.get_text().get_text().startswith("+")
            cell.set_text_props(color="#d03b3b" if worse else "#008300", fontweight="bold")


def plot_figure(model, position, nominal, calibrated, offset_mm, output):
    plt.rcParams.update({"font.family": "DejaVu Sans", "text.color": INK})
    fig = plt.figure(figsize=(12, 14), facecolor=SURFACE)
    grid = fig.add_gridspec(3, 1, height_ratios=[1, 1, 0.42], left=0.07, right=0.97, top=0.9,
                            bottom=0.085, hspace=0.42)
    n_poses = sum(len(xy) for xy in nominal["xy"])
    draw_row(fig, grid[0], "Nominal URDF", nominal, offset_mm)
    draw_row(fig, grid[1], "Calibrated URDF", calibrated, offset_mm)
    draw_table(fig.add_subplot(grid[2]), nominal, calibrated)
    fig.suptitle(f"Socket spread and distance ({model}, {position}, {n_poses} poses)", x=0.03, y=0.975,
                 ha="left", fontsize=15, fontweight="bold", color=INK)
    fig.text(0.03, 0.012,
             "Dots = predicted sphere centre of each recorded pose (FK of the URDF). + = socket centre, the "
             "mean of its dots (not measured), at the origin of each socket plot.\nCircle and arrow = σ (RMS "
             "distance to the centre); MAE = mean distance. d = distance between the two centres, ideally 50 mm.\n"
             "Both sockets of a row share the same true mm scale; the two rows have different scales.",
             fontsize=9, color=INK2, linespacing=1.5)
    fig.savefig(output, dpi=130, facecolor=SURFACE)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description="Plot socket spread and distance, nominal vs calibrated")
    parser.add_argument("--model", "-m", type=str, required=True,
                        help="Calibrated robot model, saved in the calibrated_urdf folder.")
    parser.add_argument("--data", "-t", type=str, nargs="+", default=None,
                        help="<robot-name>/<tool-position> folders in data. Default: the training data of the model.")
    parser.add_argument("--set", default=None,
                        help="Label for the file name and title (e.g. test). Default: train / val / test from the data folder.")
    args = parser.parse_args()

    parent_directory = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), os.path.pardir))
    model_folder = os.path.join(parent_directory, "calibrated_urdf", args.model)
    if not os.path.isdir(model_folder):
        print(f"model {model_folder} does not exist in calibrated_urdf. Did you already run the optimizer?")
        sys.exit(1)

    config = yaml.safe_load(open(os.path.join(model_folder, "config.yaml")))
    urdf_name = config["urdf"]
    offset_mm = float(config.get("offset_distance", 0.05)) * 1e3
    nominal_urdf = os.path.join(parent_directory, "urdf", f"{urdf_name}.urdf")
    calibrated_urdf = os.path.join(model_folder, f"{urdf_name}.urdf")

    if args.data:
        data_folders = [os.path.join(parent_directory, "data", d) for d in args.data]
    else:
        # config.yaml stores absolute paths of the machine that ran the optimizer: keep the folder name
        robot_name = config.get("robot-name", args.model)
        data_folders = [os.path.join(parent_directory, "data", robot_name, os.path.basename(p.rstrip("/")))
                        for p in config["data_path"]]

    for data_folder in data_folders:
        if not os.path.isdir(data_folder):
            print(f"data folder {data_folder} does not exist")
            sys.exit(1)
        nominal = socket_clouds(nominal_urdf, data_folder, offset_mm)
        calibrated = socket_clouds(calibrated_urdf, data_folder, offset_mm)
        split = args.set or os.path.basename(os.path.dirname(data_folder.rstrip("/")))
        suffix = f"_{split}" if (args.set or split in ("train", "val", "test")) else ""
        for position in nominal:
            output = os.path.join(model_folder, f"mae_vs_sigma_{position}{suffix}.png")
            label = f"{position} {split}" if suffix else position
            plot_figure(args.model, label, nominal[position], calibrated[position], offset_mm, output)
            print(f"Saved {output}  (d {nominal[position]['distance']:.3f} -> "
                  f"{calibrated[position]['distance']:.3f} mm)")


if __name__ == "__main__":
    main()
