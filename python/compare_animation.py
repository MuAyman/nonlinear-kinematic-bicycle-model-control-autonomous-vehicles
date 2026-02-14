# Import libraries
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# =========================
# Parameters
# =========================
RAD2DEG = 180.0 / np.pi
speed_factor = 2
dt = 0.05

# =========================
# Load data for BOTH controllers
# =========================
CTRL1_NAME = "Stanley"
CTRL2_NAME = "Pure Pursuit"
CTRL1_CSV = "results/Stanley_trajectory5.csv"
CTRL2_CSV = "results/PP_trajectory5.csv"

sim1 = pd.read_csv(CTRL1_CSV)
sim2 = pd.read_csv(CTRL2_CSV)

# Waypoints (assume same for both)
wp = sim1[["x_ref", "y_ref"]].drop_duplicates().reset_index(drop=True)
wp_x = wp["x_ref"].values
wp_y = wp["y_ref"].values

# Extract controller 1 data
t1 = sim1["t"].values
x1 = sim1["x"].values
y1 = sim1["y"].values
psi1 = sim1["psi"].values
delta1 = sim1["delta"].values
v1 = sim1["v"].values * 3.6

# Extract controller 2 data
t2 = sim2["t"].values
x2 = sim2["x"].values
y2 = sim2["y"].values
psi2 = sim2["psi"].values
delta2 = sim2["delta"].values
v2 = sim2["v"].values * 3.6

# Ensure same length
N = min(len(t1), len(t2))

# =========================
# Cross-Tracking Error for both
# =========================
def compute_cte(x, y):
    cte = np.zeros(len(x))
    for i in range(len(x)):
        dx = wp_x - x[i]
        dy = wp_y - y[i]
        cte[i] = np.round(np.sqrt(np.min(dx**2 + dy**2)), 2)
    return cte

cte1 = compute_cte(x1, y1)
cte2 = compute_cte(x2, y2)

# =========================
# Figure setup
# =========================
plt.rcParams["figure.autolayout"] = True
fig = plt.figure(figsize=(15, 5))

# Use margins so axis labels/ticks are visible (leave room for y-axis)
AX_MARGIN_LEFT = 0.05
AX_MARGIN_BOTTOM = 0.05
AX_MARGIN_RIGHT = 0.95
AX_MARGIN_TOP = 0.95
ax_main = fig.add_axes([AX_MARGIN_LEFT, AX_MARGIN_BOTTOM, AX_MARGIN_RIGHT-AX_MARGIN_LEFT, AX_MARGIN_TOP-AX_MARGIN_BOTTOM])

def on_resize(event):
    ax_main.set_position([AX_MARGIN_LEFT, AX_MARGIN_BOTTOM, AX_MARGIN_RIGHT-AX_MARGIN_LEFT, AX_MARGIN_TOP-AX_MARGIN_BOTTOM])

fig.canvas.mpl_connect("resize_event", on_resize)

ax_main.set_title("Controller Motion Comparison", fontsize=14, fontweight='bold')

# Waypoints
ax_main.plot(wp_x, wp_y, "k--", linewidth=2, label="Waypoints")

# Controller plots
path_line1, = ax_main.plot([], [], "b", lw=2, label=CTRL1_NAME)
vehicle_point1, = ax_main.plot([], [], "bo", ms=8)

path_line2, = ax_main.plot([], [], "r", lw=2, label=CTRL2_NAME)
vehicle_point2, = ax_main.plot([], [], "ro", ms=8)

info_text = ax_main.text(
    0.01, 0.01, '', transform=ax_main.transAxes,
    fontsize=11, bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
    family='monospace', fontweight='bold'
)

ax_main.set_aspect("equal")
ax_main.grid(True)
ax_main.legend()

# Determine stable axes limits (include waypoints and both trajectories)
all_x = np.concatenate([wp_x, x1[:N], x2[:N]])
all_y = np.concatenate([wp_y, y1[:N], y2[:N]])
xmin, xmax = float(np.min(all_x)), float(np.max(all_x))
ymin, ymax = float(np.min(all_y)), float(np.max(all_y))
xrange = xmax - xmin if xmax > xmin else 1.0
yrange = ymax - ymin if ymax > ymin else 1.0
# margin: 10% of the larger span plus 1m buffer
margin = max(xrange, yrange) * 0.1 + 1.0
ax_main.set_xlim(xmin - margin, xmax + margin)
ax_main.set_ylim(ymin - margin, ymax + margin)
# Disable autoscaling so animation updates don't re-zoom the axes
ax_main.set_autoscale_on(False)
ax_main.autoscale(False)

ax_main.set_xlabel("x [m]")
ax_main.set_ylabel("y [m]")

# =========================
# Update function
# =========================
def update(frame):
    # Controller 1
    path_line1.set_data(x1[:frame+1], y1[:frame+1])
    vehicle_point1.set_data([x1[frame]], [y1[frame]])

    # Controller 2
    path_line2.set_data(x2[:frame+1], y2[:frame+1])
    vehicle_point2.set_data([x2[frame]], [y2[frame]])

    # Info text
    info_text.set_text(
        f"Time: {t1[frame]:.2f} s\n"
        f"{CTRL1_NAME}: CTE={cte1[frame]:.2f} m, δ={np.degrees(delta1[frame]):.2f}°\n"
        f"{CTRL2_NAME}: CTE={cte2[frame]:.2f} m, δ={np.degrees(delta2[frame]):.2f}°"
    )

    return path_line1, vehicle_point1, path_line2, vehicle_point2, info_text

# =========================
# Run animation
# =========================
frame_interval_ms = (dt / speed_factor) * 1000
save_fps = int(1.0 / (dt / speed_factor))

ani = FuncAnimation(
    fig,
    update,
    frames=N,
    interval=frame_interval_ms,
    blit=False,
    repeat=False
)

# plt.show()

# =========================
# Save MP4
# =========================
print("Saving animation as MP4...")
ani.save("controller_comparison.mp4",
         writer='ffmpeg',
         fps=save_fps,
         dpi=100,
         bitrate=1800)
print(f"✓ Saved controller_comparison.mp4 at {save_fps} FPS")
