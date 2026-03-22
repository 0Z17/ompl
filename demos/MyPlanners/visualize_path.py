#!/usr/bin/env python3
"""
可视化规划结果：NURBS曲面 + 途径点 + 接触路径
用法: python3 visualize_path.py [output_dir] [planner_tag]
"""

import sys
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

# ---- 参数 ----
OUTPUT_DIR  = sys.argv[1] if len(sys.argv) > 1 else "/home/wsl/proj/T_mech_R1/S2"
PLANNER_TAG = sys.argv[2] if len(sys.argv) > 2 else "AtlasRRTstar"
GRID_N      = 20  # 与 C++ 中 SURFACE_GRID_N 一致

surface_file   = os.path.join(OUTPUT_DIR, "surface_grid.csv")
waypoints_file = os.path.join(OUTPUT_DIR, "waypoints.csv")
contact_file   = os.path.join(OUTPUT_DIR, f"path_contact_{PLANNER_TAG}.csv")

# ---- 读取数据 ----
surf = pd.read_csv(surface_file)
wpts = pd.read_csv(waypoints_file)
path = pd.read_csv(contact_file)

# ---- 重塑曲面网格 ----
n = GRID_N + 1
X = surf["x"].values.reshape(n, n)
Y = surf["y"].values.reshape(n, n)
Z = surf["z"].values.reshape(n, n)

# ---- 等比例坐标轴辅助函数 ----
def set_axes_equal(ax):
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()
    x_range = x_limits[1] - x_limits[0]
    y_range = y_limits[1] - y_limits[0]
    z_range = z_limits[1] - z_limits[0]
    max_range = max(x_range, y_range, z_range)
    x_mid = np.mean(x_limits)
    y_mid = np.mean(y_limits)
    z_mid = np.mean(z_limits)
    ax.set_xlim3d([x_mid - max_range/2, x_mid + max_range/2])
    ax.set_ylim3d([y_mid - max_range/2, y_mid + max_range/2])
    ax.set_zlim3d([z_mid - max_range/2, z_mid + max_range/2])

# ---- 绘图 ----
fig = plt.figure(figsize=(13, 10))
ax = fig.add_subplot(111, projection="3d")
ax.set_facecolor("#f0f0f0")
fig.patch.set_facecolor("white")

# 1. 曲面：单色 + 网格线，对比背景清晰
surf_plot = ax.plot_surface(
    X, Y, Z,
    color="steelblue", alpha=0.6,
    linewidth=0.3, edgecolor="dimgray",
    antialiased=True
)

# 2. 途径点
ax.scatter(
    wpts["x"].values, wpts["y"].values, wpts["z"].values,
    color="red", s=100, zorder=5, label="Waypoints", depthshade=False
)
for _, row in wpts.iterrows():
    ax.text(row["x"], row["y"], row["z"],
            f" ({row['u']:.1f},{row['v']:.1f})",
            fontsize=7, color="darkred")

# 3. 接触路径
ax.plot(
    path["x"].values, path["y"].values, path["z"].values,
    color="orangered", linewidth=2.5, label="Contact path", zorder=6
)
ax.scatter(
    path["x"].iloc[0], path["y"].iloc[0], path["z"].iloc[0],
    color="lime", s=80, zorder=7, label="Start", depthshade=False
)
ax.scatter(
    path["x"].iloc[-1], path["y"].iloc[-1], path["z"].iloc[-1],
    color="magenta", s=80, zorder=7, label="Goal", depthshade=False
)

# 4. 法向量（每隔若干点画一个箭头）
step = max(1, len(path) // 30)
scale = 0.08
for i in range(0, len(path), step):
    r = path.iloc[i]
    ax.quiver(r["x"], r["y"], r["z"],
              r["nx"] * scale, r["ny"] * scale, r["nz"] * scale,
              color="steelblue", linewidth=0.8, arrow_length_ratio=0.3)

ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_zlabel("Z (m)")
ax.set_title(f"Surface + Path ({PLANNER_TAG})", fontsize=13)
ax.legend(loc="upper left")

set_axes_equal(ax)
plt.tight_layout()

out_png = os.path.join(OUTPUT_DIR, f"visualization_{PLANNER_TAG}.png")
plt.savefig(out_png, dpi=150)
print(f"Saved: {out_png}")
plt.show()
