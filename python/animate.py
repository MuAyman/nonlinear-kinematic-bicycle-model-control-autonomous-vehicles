import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

# =========================
# Load simulation data
# =========================
sim = pd.read_csv("results/PP_trajectory6.csv")
# Columns:
# t, x_ref, y_ref, x, y, psi, delta, v, delta_dot

t = sim["t"].values
x_ref = sim["x_ref"].values
y_ref = sim["y_ref"].values
x = sim["x"].values
y = sim["y"].values
psi = sim["psi"].values
delta = sim["delta"].values

# =========================
# Compute cross-tracking error
# (distance to current reference point)
# =========================
cte = np.sqrt((x - x_ref) ** 2 + (y - y_ref) ** 2)

# =========================
# Figure & layout
# =========================
plt.style.use("seaborn-v0_8-darkgrid")

fig = plt.figure(figsize=(12, 8))
gs = fig.add_gridspec(3, 2, height_ratios=[2.5, 1, 1])

ax_traj = fig.add_subplot(gs[:, 0])
ax_cte = fig.add_subplot(gs[0, 1])
ax_delta = fig.add_subplot(gs[1, 1])
ax_empty = fig.add_subplot(gs[2, 1])
ax_empty.axis("off")

# =========================
# Trajectory plot
# =========================
ax_traj.set_title("Vehicle Motion (Pure Pursuit)")
ax_traj.set_xlabel("X [m]")
ax_traj.set_ylabel("Y [m]")
ax_traj.plot(x_ref, y_ref, "k--", linewidth=2, label="Reference Path")

veh_path, = ax_traj.plot([], [], "b-", linewidth=2, label="Vehicle Path")
veh_point, = ax_traj.plot([], [], "bo", markersize=8)

heading_arrow = ax_traj.arrow(
    x[0], y[0], np.cos(psi[0]), np.sin(psi[0]),
    head_width=0.3, head_length=0.5, fc="r", ec="r"
)

ax_traj.legend()
ax_traj.set_aspect("equal", adjustable="box")

# =========================
# Cross-tracking error plot
# =========================
ax_cte.set_title("Cross-Tracking Error")
ax_cte.set_xlabel("Time [s]")
ax_cte.set_ylabel("CTE [m]")
ax_cte.set_xlim(t[0], t[-1])
ax_cte.set_ylim(0, 1.2 * np.max(cte))

cte_line, = ax_cte.plot([], [], "r", linewidth=2)

# =========================
# Steering angle plot
# =========================
ax_delta.set_title("Steering Angle")
ax_delta.set_xlabel("Time [s]")
ax_delta.set_ylabel("δ [rad]")
ax_delta.set_xlim(t[0], t[-1])
ax_delta.set_ylim(
    1.2 * np.min(delta),
    1.2 * np.max(delta)
)

delta_line, = ax_delta.plot([], [], "g", linewidth=2)

# =========================
# Initialization
# =========================
def init():
    veh_path.set_data([], [])
    veh_point.set_data([], [])
    cte_line.set_data([], [])
    delta_line.set_data([], [])
    return veh_path, veh_point, cte_line, delta_line

# =========================
# Animation update
# =========================
def update(i):
    global heading_arrow

    # Vehicle trajectory
    veh_path.set_data(x[:i+1], y[:i+1])
    veh_point.set_data(x[i], y[i])

    # Heading arrow
    heading_arrow.remove()
    heading_arrow = ax_traj.arrow(
        x[i], y[i],
        1.5 * np.cos(psi[i]),
        1.5 * np.sin(psi[i]),
        head_width=0.3,
        head_length=0.5,
        fc="r",
        ec="r"
    )

    # Cross-tracking error
    cte_line.set_data(t[:i+1], cte[:i+1])

    # Steering angle
    delta_line.set_data(t[:i+1], delta[:i+1])

    return veh_path, veh_point, cte_line, delta_line

# =========================
# Create animation
# =========================
ani = FuncAnimation(
    fig,
    update,
    frames=len(t),
    init_func=init,
    interval=40,
    blit=False
)

# =========================
# Save as GIF
# =========================
writer = PillowWriter(fps=25)
ani.save("pure_pursuit_animation.gif", writer=writer)

plt.close(fig)
