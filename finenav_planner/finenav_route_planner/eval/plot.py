#!/usr/bin/env python3
"""Visualise Route Planner standalone eval output.

Usage:
    python3 eval/plot.py [OUTPUT_DIR]

Default: /tmp/dstarlite_eval  (mirrors cfg::kOutputDir)

Reads paths.csv and stats.csv produced by route_planner_eval,
reconstructs the grid from constants matching eval_config.hpp,
and renders a multi-panel PNG figure.

Panel layout depends on the scenario detected in stats.csv:
  Scenario 1 (empty_diagonal):   1×2 — A* vs D* Lite side-by-side
  Scenario 2 (wall_replan):      2×2 — initial + replan for each algorithm
  Scenario 3 (terrain_sweep):    1×1 — terrain band heatmap + all weight paths

Requires: numpy, pandas, matplotlib >= 3.5
"""

#  Copyright (c) 2026.
#  IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
#  All rights reserved.

import math
import os
import sys

import matplotlib.gridspec as gridspec
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.lines import Line2D

# ── Default output dir (mirrors cfg::kOutputDir) ────────────────────────────
DEFAULT_DIR = "/tmp/dstarlite_eval"

# ── Grid config constants — auto-loaded from _config.json ─────────────────────
GRID_SIZE       = 100
KWallX          = 50
KGapY0, KGapY1  = 30, 70
KClosureY0, KClosureY1 = 45, 55
KBandX0, KBandX1 = 35, 45
KBandY0, KBandY1 = 10, 89
KTerrainBandCost = 10.0


def _load_config(out_dir):
    """Read eval_config constants from _config.json, falling back to defaults."""
    import json as _json
    cfg_path = os.path.join(out_dir, "_config.json")
    if not os.path.exists(cfg_path):
        return
    try:
        with open(cfg_path) as f:
            d = _json.load(f)
        global GRID_SIZE, KWallX, KGapY0, KGapY1, KClosureY0, KClosureY1
        global KBandX0, KBandX1, KBandY0, KBandY1, KTerrainBandCost
        GRID_SIZE       = d.get("grid_size", GRID_SIZE)
        KWallX          = d.get("wall_x", KWallX)
        KGapY0          = d.get("gap_y0", KGapY0)
        KGapY1          = d.get("gap_y1", KGapY1)
        KClosureY0      = d.get("closure_y0", KClosureY0)
        KClosureY1      = d.get("closure_y1", KClosureY1)
        KBandX0         = d.get("band_x0", KBandX0)
        KBandX1         = d.get("band_x1", KBandX1)
        KBandY0         = d.get("band_y0", KBandY0)
        KBandY1         = d.get("band_y1", KBandY1)
        KTerrainBandCost = d.get("band_cost", KTerrainBandCost)
    except Exception:
        pass  # keep defaults

# ── Colour palette ──────────────────────────────────────────────────────────
ALGO_COLORS = {
    "AStar":     "#2196F3",   # blue
    "DStarLite": "#FF5722",   # deep orange
}
PATH_COLORS = {
    "initial": "#1565C0",     # dark blue
    "replan":  "#C62828",     # dark red
}
# Terrain sweep: blue (low tw) → red (high tw)
SWEEP_CMAP = plt.cm.coolwarm


# ─────────────────────────────────────────────────────────────────────────────
# Grid builders (replicating eval_scenario.cpp logic)
# ─────────────────────────────────────────────────────────────────────────────

def build_empty_grid():
    """Empty grid — all cells walkable, cost 0."""
    return np.zeros((GRID_SIZE, GRID_SIZE))


def build_wall_grid(with_closure=False):
    """Wall with a gap (Scenario 2).

    Two wall segments at x=KWallX:
      bottom: y ∈ [0, KGapY0]
      top:    y ∈ [KGapY1, 99]
    If with_closure=True, also draws the sealing bar
    x∈[KWallX-1, KWallX+1], y∈[KClosureY0, KClosureY1]
    that forces the replan detour.
    """
    grid = np.zeros((GRID_SIZE, GRID_SIZE))
    grid[:KGapY0 + 1, KWallX] = 1.0              # bottom segment
    grid[KGapY1:, KWallX] = 1.0                   # top segment
    if with_closure:
        grid[KClosureY0:KClosureY1 + 1, KWallX - 1:KWallX + 2] = 1.0
    return grid


def build_terrain_band_grid():
    """Grid with high-cost band in the centre."""
    grid = np.zeros((GRID_SIZE, GRID_SIZE))
    grid[KBandY0:KBandY1 + 1, KBandX0:KBandX1 + 1] = KTerrainBandCost
    return grid


# ─────────────────────────────────────────────────────────────────────────────
# Plotting helpers
# ─────────────────────────────────────────────────────────────────────────────

def _add_stats_text(ax, stats_df, algo=None, label=None):
    """Overlay statistics text box on a panel."""
    lines = []
    subset = stats_df
    if algo is not None:
        subset = subset[subset["algo"] == algo]
    if label is not None:
        subset = subset[subset["path_type"] == label]

    for _, row in subset.iterrows():
        lines.append(
            f"{row['algo']} | {row['path_type']}: "
            f"len={int(row['length'])}  "
            f"{row['compute_ms']:.2f} ms  "
            f"cost={row['total_terrain_cost']:.0f}"
        )

    text = "\n".join(lines) if lines else "no data"
    ax.text(
        0.02, 0.98, text,
        transform=ax.transAxes,
        fontsize=10, fontfamily="monospace",
        verticalalignment="top",
        bbox=dict(boxstyle="round,pad=0.3", facecolor="wheat", alpha=0.85),
        zorder=10,
    )


def _draw_grid(ax, grid, cmap=None, vmin=None, vmax=None):
    """Render grid as an image with optional colour map."""
    if cmap is None:
        # Obstacle grid: black = obstacle, white = free
        cmap = LinearSegmentedColormap.from_list("obs", ["white", "black"])
        ax.imshow(grid, origin="lower", cmap=cmap, vmin=0, vmax=1,
                  extent=[0, GRID_SIZE, 0, GRID_SIZE])
    else:
        im = ax.imshow(grid, origin="lower", cmap=cmap, vmin=vmin, vmax=vmax,
                       extent=[0, GRID_SIZE, 0, GRID_SIZE])
        return im


def _draw_path(ax, df, algo, label, color, lw=2.0, ls="-", marker="o", ms=4):
    """Draw a single path on the given axes."""
    sub = df[(df["algo"] == algo) & (df["label"] == label)]
    if sub.empty:
        return
    # Sort by step to ensure correct order
    sub = sub.sort_values("step")
    ax.plot(sub["x"].to_numpy(), sub["y"].to_numpy(),
            color=color, lw=lw, ls=ls,
            marker=marker, ms=ms, alpha=0.85, zorder=5)


def _mark_endpoints(ax, start=(0,0), goal=(GRID_SIZE - 1, GRID_SIZE - 1)):
    """Mark start (green triangle) and goal (red star)."""
    ax.plot(start[0], start[1], "g^", ms=14, zorder=8, clip_on=False)
    ax.plot(goal[0],  goal[1],  "r*", ms=16, zorder=8, clip_on=False)


def _stylise(ax, title):
    """Common axis styling."""
    ax.set_xlim(-0.5, GRID_SIZE - 0.5)
    ax.set_ylim(-0.5, GRID_SIZE - 0.5)
    ax.set_aspect("equal")
    ax.set_title(title, fontsize=12)
    ax.grid(False)


# ─────────────────────────────────────────────────────────────────────────────
# Per-scenario plot functions
# ─────────────────────────────────────────────────────────────────────────────

def plot_scenario1(paths_df, stats_df, out_dir):
    """Empty diagonal — A* vs D* Lite side-by-side."""
    grid = build_empty_grid()
    fig, (ax_a, ax_d) = plt.subplots(1, 2, figsize=(14, 7))
    fig.suptitle("Scenario 1: Empty Grid Diagonal  —  A* vs D* Lite",
                 fontsize=14, fontweight="bold")

    for ax, algo in [(ax_a, "AStar"), (ax_d, "DStarLite")]:
        _draw_grid(ax, grid)
        _draw_path(ax, paths_df, algo, "initial", ALGO_COLORS[algo], lw=1.5)
        _mark_endpoints(ax)
        _stylise(ax, f"{algo}")
        _add_stats_text(ax, stats_df, algo=algo)

    plt.show()


def plot_scenario2(paths_df, stats_df, out_dir):
    """Two-wall gap — initial through gap, replan goes around closure."""
    grid_init  = build_wall_grid(with_closure=False)
    grid_replan = build_wall_grid(with_closure=True)
    fig, axes = plt.subplots(2, 2, figsize=(14, 13))
    fig.suptitle("Scenario 2: Wall Gap  —  initial vs replan (closure sealed)",
                 fontsize=14, fontweight="bold")

    panels = [
        (axes[0, 0], "DStarLite", "initial",  grid_init),
        (axes[0, 1], "DStarLite", "replan",   grid_replan),
        (axes[1, 0], "AStar",      "initial",  grid_init),
        (axes[1, 1], "AStar",      "replan",   grid_replan),
    ]

    for ax, algo, label, grid in panels:
        _draw_grid(ax, grid)
        color = PATH_COLORS.get(label, "gray")
        ls = "--" if label == "initial" else "-"
        _draw_path(ax, paths_df, algo, label, color, lw=1.8, ls=ls)
        _mark_endpoints(ax, start=(0, 50), goal=(99, 50))
        _stylise(ax, f"{algo} — {label}")
        _add_stats_text(ax, stats_df, algo=algo, label=label)

    plt.show()


def plot_scenario3(paths_df, stats_df, out_dir):
    """Terrain band weight sweep — all paths on one heatmap."""
    grid = build_terrain_band_grid()
    fig, (ax_grid, ax_time) = plt.subplots(1, 2, figsize=(16, 7))
    fig.suptitle("Scenario 3: Terrain Band Weight Sweep",
                 fontsize=14, fontweight="bold")

    # ── Left: grid with all paths overlaid ──
    im = _draw_grid(ax_grid, grid,
                    cmap=plt.cm.YlOrRd, vmin=0, vmax=KTerrainBandCost)
    cbar = plt.colorbar(im, ax=ax_grid, shrink=0.75,
                         label="Terrain cost (raw)")
    cbar.ax.yaxis.label.set_size(11)

    # Get unique weight labels in order
    labels = stats_df["path_type"].unique()
    n_labels = len(labels)

    for i, label in enumerate(labels):
        color = SWEEP_CMAP(i / max(n_labels - 1, 1))
        sub = stats_df[stats_df["path_type"] == label]
        algo = sub["algo"].iloc[0] if len(sub) > 0 else "?"
        _draw_path(ax_grid, paths_df, algo, label, color, lw=1.5, ls="-", ms=2)

    _mark_endpoints(ax_grid, start=(0, 50), goal=(99, 50))
    _stylise(ax_grid, "Paths by terrain_weight")

    # Legend
    legend_handles = [
        Line2D([0], [0], color=SWEEP_CMAP(i / max(n_labels - 1, 1)),
               lw=2, label=label)
        for i, label in enumerate(labels)
    ]
    ax_grid.legend(handles=legend_handles, fontsize=10, loc="lower left",
                   title="terrain_weight", title_fontsize=11)

    # ── Right: compute time vs terrain_weight ──
    stats_algo = stats_df[stats_df["algo"] == "DStarLite"]
    tw_vals = [float(l.replace("tw=", "")) for l in stats_algo["path_type"]]
    times   = stats_algo["compute_ms"].values
    lengths  = stats_algo["length"].values

    ax_time.plot(tw_vals, times, "o-", color="#FF5722", lw=1.5, ms=6,
                 label="D* Lite time")
    ax_time.set_xlabel("terrain_weight", fontsize=11)
    ax_time.set_ylabel("compute time [ms]", fontsize=11, color="#FF5722")
    ax_time.set_xscale("log")
    ax_time.grid(True, alpha=0.3)

    ax_len = ax_time.twinx()
    ax_len.plot(tw_vals, lengths, "s--", color="#2196F3", lw=1.5, ms=6,
                label="path length")
    ax_len.set_ylabel("path length [cells]", fontsize=11, color="#2196F3")

    # Combine legends
    lines1, labels1 = ax_time.get_legend_handles_labels()
    lines2, labels2 = ax_len.get_legend_handles_labels()
    ax_time.legend(lines1 + lines2, labels1 + labels2, fontsize=10,
                   loc="upper left")

    ax_time.set_title("Compute time & path length vs terrain_weight",
                      fontsize=12)

    plt.show()


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main():
    out_dir = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_DIR

    paths_path = os.path.join(out_dir, "paths.csv")
    stats_path = os.path.join(out_dir, "stats.csv")

    if not os.path.exists(paths_path) or not os.path.exists(stats_path):
        print(f"[ERROR] CSV files not found in: {out_dir}")
        print("  Expected: paths.csv and stats.csv")
        print("  Run the eval first:")
        print("    ./build/finenav_route_planner/eval/route_planner_eval")
        sys.exit(1)

    _load_config(out_dir)

    paths_df = pd.read_csv(paths_path)
    stats_df = pd.read_csv(stats_path)

    scenario = stats_df["scenario"].iloc[0] if len(stats_df) > 0 else "unknown"
    print(f"[plot]  Scenario: {scenario}  |  "
          f"{len(paths_df)} waypoints  |  {len(stats_df)} stats")

    if scenario == "empty_diagonal":
        plot_scenario1(paths_df, stats_df, out_dir)
    elif scenario == "wall_replan":
        plot_scenario2(paths_df, stats_df, out_dir)
    elif scenario == "terrain_sweep":
        plot_scenario3(paths_df, stats_df, out_dir)
    else:
        print(f"[ERROR] Unknown scenario: {scenario}")
        sys.exit(1)



if __name__ == "__main__":
    main()
