"""Schematic of the two MUKCa metrics: consistency (sigma) and distortion (epsilon).

Top: the ideal tool, two socket points 50 mm apart.
Bottom: the FK-predicted sphere centres of the recorded poses, scattered around each socket centre
(the mean of its points); sigma = spread around the centre, epsilon = |d - 50 mm| with d the distance
between the two centres.

Writes socket_metrics_schematic.{png,pdf} next to this script.
"""
import os

import numpy as np
import matplotlib as mpl
mpl.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import font_manager  # noqa: E402
from matplotlib.patches import Circle, FancyArrowPatch  # noqa: E402

INK, MUTED = '#1F2430', '#6B7280'
S0, S1 = '#4C6EF5', '#E4572E'   # socket 0 / socket 1
# Text in Lato (DejaVu Sans fallback); math in Computer Modern (LaTeX look) via mathtext.
fams = {f.name for f in font_manager.fontManager.ttflist}
mpl.rcParams.update({'font.family': 'Lato' if 'Lato' in fams else 'DejaVu Sans', 'font.size': 12,
                     'text.color': INK, 'pdf.fonttype': 42, 'mathtext.fontset': 'cm'})

OUT = os.path.dirname(os.path.abspath(__file__))

L = 10.0                  # ideal socket distance (50 mm), drawn length
Y_IDEAL, Y_MEAS = 3.2, 0.0
EPS = 0.9                 # exaggerated distance error: measured d = L + EPS
R_SIGMA = 0.85            # sigma circle radius (exaggerated)


def arrow(ax, p, q, color=INK, style='<|-|>', lw=1.3, ms=13, z=6):
    ax.add_patch(FancyArrowPatch(p, q, arrowstyle=style, mutation_scale=ms, color=color, lw=lw,
                                 shrinkA=0, shrinkB=0, zorder=z))


def main():
    rng = np.random.default_rng(3)
    fig, ax = plt.subplots(figsize=(12, 5.0))
    ax.set_aspect('equal')
    ax.axis('off')

    # ---- top: ideal tool, two points L apart -------------------------------------------------
    ideal = [np.array([0.0, Y_IDEAL]), np.array([L, Y_IDEAL])]
    ax.plot([ideal[0][0], ideal[1][0]], [Y_IDEAL, Y_IDEAL], color=INK, lw=2.2, zorder=2)
    for p, col, name in zip(ideal, (S0, S1), ('socket 0', 'socket 1')):
        ax.scatter(*p, s=110, facecolor='white', edgecolor=col, lw=2.4, zorder=5)
        ax.text(p[0], p[1] + 0.42, name, ha='center', va='bottom', fontsize=12, fontweight='bold', color=col)
    ax.text(L / 2, Y_IDEAL + 0.22, r'$d = 50$ [mm]', ha='center', va='bottom', fontsize=14, color=INK)
    ax.text(-1.55, Y_IDEAL, 'ideal', ha='right', va='center', fontsize=12, fontweight='bold', color=MUTED)

    # ---- bottom: FK-predicted sphere centres scattered around each socket centre -------------
    centres = [np.array([0.0, Y_MEAS]), np.array([L + EPS, Y_MEAS])]
    for k, (c, col) in enumerate(zip(centres, (S0, S1))):
        ax.add_patch(Circle(c, R_SIGMA, fill=True, fc=col, alpha=0.08, ec='none', zorder=1))
        ax.add_patch(Circle(c, R_SIGMA, fill=False, ls='--', lw=1.4, ec=INK, zorder=2))
        n = 11
        ang = rng.uniform(0, 2 * np.pi, n)
        rad = R_SIGMA * np.sqrt(rng.uniform(0.04, 0.9, n))
        off = np.column_stack([rad * np.cos(ang), rad * np.sin(ang)])
        pts = c + off - off.mean(0)          # the points' mean is exactly the centre
        ax.scatter(pts[:, 0], pts[:, 1], s=34, color=col, edgecolor='white', lw=0.8, zorder=6)
        ax.scatter(*c, s=110, marker='x', color=col, lw=2.6, zorder=8)
        # mu: socket centre = mean of the points
        side = -1 if k == 0 else 1
        ax.annotate(rf'$\mu_{k}$', xy=c, xytext=(c[0] + side * 1.35, c[1] - 0.75), fontsize=18, color=col,
                    ha='center', va='center', zorder=9,
                    arrowprops={'arrowstyle': '-', 'color': col, 'lw': 0.9, 'shrinkA': 4, 'shrinkB': 6})
        # sigma: radius arrow
        a = np.deg2rad(125 if k == 0 else 55)
        tip = c + R_SIGMA * np.array([np.cos(a), np.sin(a)])
        arrow(ax, c, tip, style='-|>', lw=1.1, ms=12, z=7)
        ax.text(tip[0] + (-0.18 if k == 0 else 0.18), tip[1] + 0.08, rf'$\sigma_{k}$', ha='center', va='bottom',
                fontsize=20, color=INK)
    ax.text(-1.55, Y_MEAS, 'measured', ha='right', va='center', fontsize=12, fontweight='bold', color=MUTED)

    # d: distance between the two socket centres
    y_d = Y_MEAS - R_SIGMA - 0.45
    ax.plot([0.0, 0.0], [Y_IDEAL - 0.18, y_d - 0.15], color=MUTED, lw=0.9, ls=(0, (3, 2.5)), zorder=1)
    ax.plot([centres[1][0]] * 2, [Y_MEAS - 0.12, y_d - 0.15], color=MUTED, lw=0.9, ls=(0, (3, 2.5)), zorder=1)
    arrow(ax, (centres[0][0], y_d), (centres[1][0], y_d), color=INK)
    ax.text(L / 2, y_d - 0.12, r'$d + \epsilon$', ha='center', va='top', fontsize=18, color=INK)

    # epsilon: gap between the ideal 50 mm and d, at socket 1
    ax.plot([ideal[1][0], ideal[1][0]], [Y_IDEAL - 0.18, y_d - 0.15], color=MUTED, lw=0.9,
            ls=(0, (3, 2.5)), zorder=1)
    y_e = y_d - 0.75
    for x in (ideal[1][0], centres[1][0]):
        ax.plot([x, x], [y_d - 0.15, y_e - 0.15], color=MUTED, lw=0.9, ls=(0, (3, 2.5)), zorder=1)
    arrow(ax, (ideal[1][0], y_e), (centres[1][0], y_e), color=S1, lw=1.4, ms=11)
    ax.text((ideal[1][0] + centres[1][0]) / 2, y_e - 0.14, r'$\epsilon$', ha='center', va='top', fontsize=20,
            color=S1)

    ax.set_xlim(-3.0, L + EPS + 1.4)
    ax.set_ylim(y_e - 0.9, Y_IDEAL + 1.1)
    for ext in ('png', 'pdf'):
        fig.savefig(os.path.join(OUT, f'socket_metrics_schematic.{ext}'), dpi=220, bbox_inches='tight')
    plt.close(fig)
    print('saved', os.path.join(OUT, 'socket_metrics_schematic') + '.{png,pdf}')


if __name__ == '__main__':
    main()
