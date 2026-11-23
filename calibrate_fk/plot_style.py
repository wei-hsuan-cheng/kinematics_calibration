"""Shared plot style: Lato text (DejaVu Sans fallback), LaTeX-like math (Computer Modern), muted axes, light grid."""
import matplotlib as mpl
from matplotlib import font_manager

INK, INK2, MUTED, GRID = "#1F2430", "#4B5563", "#6B7280", "#E5E7EB"
SURFACE = "white"
# categorical colors, one per tool position (first two are also the socket 0 / 1 colors)
PALETTE = ["#4C6EF5", "#E4572E", "#17BEBB", "#F59F00", "#7950F2", "#2B8A3E", "#D6336C", "#2B2D6E"]
DPI = 220


def setup_style():
    fams = {f.name for f in font_manager.fontManager.ttflist}
    mpl.rcParams.update({
        "font.family": "Lato" if "Lato" in fams else "DejaVu Sans",
        "font.size": 10, "axes.titlesize": 11, "axes.titleweight": "bold", "axes.labelcolor": INK,
        "axes.labelsize": 10, "axes.edgecolor": "#9CA3AF", "axes.linewidth": 0.8,
        "axes.spines.top": False, "axes.spines.right": False,
        "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.7,
        "xtick.color": MUTED, "ytick.color": MUTED, "xtick.labelsize": 9, "ytick.labelsize": 9,
        "text.color": INK, "legend.frameon": False, "legend.fontsize": 9,
        "figure.facecolor": SURFACE, "savefig.facecolor": SURFACE, "pdf.fonttype": 42,
        "mathtext.fontset": "cm",
    })


def position_colors(names):
    """Same color for the same tool position across plots."""
    return {name: PALETTE[i % len(PALETTE)] for i, name in enumerate(names)}
