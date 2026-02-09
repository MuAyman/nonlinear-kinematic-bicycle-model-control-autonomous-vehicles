import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# =========================
# Physical limits
# =========================
MAX_VELOCITY = 15.0 * 3.6
MIN_VELOCITY = -5.0 * 3.6

MAX_STEERING_RATE_RAD = 0.5
MIN_STEERING_RATE_RAD = -0.5

MAX_STEERING_ANGLE_RAD = np.pi / 4
MIN_STEERING_ANGLE_RAD = -np.pi / 4

RAD2DEG = 180.0 / np.pi

MAX_STEERING_RATE_DEG = MAX_STEERING_RATE_RAD * RAD2DEG
MIN_STEERING_RATE_DEG = MIN_STEERING_RATE_RAD * RAD2DEG
MAX_STEERING_ANGLE_DEG = MAX_STEERING_ANGLE_RAD * RAD2DEG
MIN_STEERING_ANGLE_DEG = MIN_STEERING_ANGLE_RAD * RAD2DEG

# =========================
# Load data
# =========================
sim = pd.read_csv("results/Stanley/Stanley_trajectory5.csv")
# sim = pd.read_csv("results/PurePursuit/PP_trajectory5.csv")

# t,x_ref,y_ref,x,y,psi,delta,v,delta_dot
wp = sim[["x_ref", "y_ref"]].drop_duplicates().reset_index(drop=True)

required_sim_cols = {"t", "x", "y", "psi", "delta", "v", "delta_dot"}
required_wp_cols = {"x_ref", "y_ref"}

if not required_sim_cols.issubset(sim.columns):
    raise ValueError("simulation.csv is missing required columns")

if not required_wp_cols.issubset(wp.columns):
    raise ValueError("waypoints.csv is missing required columns")

sim = sim.sort_values("t").reset_index(drop=True)

# =========================
# Unit conversions
# =========================
t = sim["t"].values
x = sim["x"].values
y = sim["y"].values

psi_deg = np.unwrap(sim["psi"].values) * RAD2DEG
psi_rad = np.unwrap(sim["psi"].values)

delta_deg = sim["delta"].values * RAD2DEG
delta_dot_deg = sim["delta_dot"].values * RAD2DEG

v = sim["v"].values

wp_x = wp["x_ref"].values
wp_y = wp["y_ref"].values

# =========================
# Tracking error computation
# =========================
cte = np.zeros(len(x))

for i in range(len(x)):
    dx = wp_x - x[i]
    dy = wp_y - y[i]
    # round the CTE to only 2 decimal places for better visualization
    cte[i] = np.round(np.sqrt(np.min(dx**2 + dy**2)), 2)

# =========================
# FIGURE 1: Path tracking (XY only)
# =========================
plt.figure(figsize=(8, 8))
plt.plot(wp_x, wp_y, "k--", linewidth=2, label="Waypoints")
plt.plot(x, y, "b", linewidth=2, label="Vehicle Path")
plt.scatter(x[0], y[0], c="green", s=80, label="Start")
plt.scatter(x[-1], y[-1], c="red", s=80, label="End")

plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.title("Path Tracking (XY)")
plt.axis("equal")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

# =========================
# FIGURE 2: Controller behavior dashboard
# =========================
fig, axs = plt.subplots(3, 2, figsize=(16, 14))
fig.suptitle("Controller Behavior - Time-Domain Analysis", fontsize=16)

# -------------------------
# Heading vs time
# -------------------------
ax = axs[0, 0]
ax.plot(t, psi_deg, linewidth=2)
# ax.set_xlabel("Time [s]")
ax.set_ylabel("Heading ψ [deg]")
ax.set_title("Heading vs Time")
ax.grid(True)

# -------------------------
# Steering angle vs time
# -------------------------
ax = axs[0, 1]
ax.plot(t, delta_deg, linewidth=2, label="Steering angle")
ax.axhline(MAX_STEERING_ANGLE_DEG, color="red", linestyle="--", label="Steering limits")
ax.axhline(MIN_STEERING_ANGLE_DEG, color="red", linestyle="--")
# ax.set_xlabel("Time [s]")
ax.set_ylabel("Steering angle δ [deg]")
ax.set_title("Steering Angle vs Time")
ax.grid(True)
ax.legend()

# -------------------------
# Steering rate vs time
# -------------------------
ax = axs[1, 0]
ax.plot(t, delta_dot_deg, linewidth=2, label="Steering rate")
ax.axhline(MAX_STEERING_RATE_DEG, color="red", linestyle="--", label="Steering-rate limits")
ax.axhline(MIN_STEERING_RATE_DEG, color="red", linestyle="--")
# ax.set_xlabel("Time [s]")
ax.set_ylabel("Steering rate δ̇ [deg/s]")
ax.set_title("Steering Rate vs Time")
ax.grid(True)
ax.legend()

# -------------------------
# Velocity vs time
# -------------------------
ax = axs[1, 1]
ax.plot(t, v * 3.6, linewidth=2, label="Velocity")
ax.axhline(MAX_VELOCITY, color="red", linestyle="--", label="Velocity limits")
ax.axhline(MIN_VELOCITY, color="red", linestyle="--")
# ax.set_xlabel("Time [s]")
ax.set_ylabel("Velocity [km/h]")
ax.set_title("Velocity vs Time")
ax.grid(True)
ax.legend()

# -------------------------
# Tracking error vs time
# -------------------------
ax = axs[2, 0]
ax.plot(t, cte, linewidth=2)
# ax.set_xlabel("Time [s]")
ax.set_ylabel("Tracking Error [m]")
ax.set_title("Tracking Error vs Time")
ax.grid(True)

# -------------------------
# Heading error vs time
# -------------------------
ax = axs[2, 1]

# Compute path segment headings from waypoints
if len(wp_x) > 1:
    wp_dx = np.diff(wp_x)
    wp_dy = np.diff(wp_y)
    seg_angles = np.arctan2(wp_dy, wp_dx)  # length = len(wp)-1

    psi_ref = np.zeros(len(x))
    for i_pos in range(len(x)):
        dx = wp_x - x[i_pos]
        dy = wp_y - y[i_pos]
        idx = np.argmin(dx**2 + dy**2)
        seg_idx = min(idx, len(seg_angles) - 1)
        psi_ref[i_pos] = seg_angles[seg_idx]

    # heading error (vehicle - path) wrapped to [-pi, pi]
    heading_err_rad = np.arctan2(np.sin(psi_rad - psi_ref), np.cos(psi_rad - psi_ref))
    heading_err_deg = heading_err_rad * RAD2DEG

    ax.plot(t, heading_err_deg, linewidth=2)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Heading error [deg]")
    ax.set_title("Heading Error vs Time")
    ax.grid(True)
else:
    # no waypoints to compute heading reference
    ax.text(0.5, 0.5, "No waypoints for heading reference", ha="center", va="center")
    ax.axis("off")

plt.tight_layout(rect=[0, 0, 1, 0.95])
plt.show()
