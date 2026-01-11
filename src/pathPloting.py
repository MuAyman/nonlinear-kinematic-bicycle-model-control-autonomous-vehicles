import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# =========================
# Physical limits
# =========================
MAX_VELOCITY = 30.0      # m/s
MIN_VELOCITY = -10.0     # m/s

MAX_STEERING_RATE_RAD = 0.5   # rad/s
MIN_STEERING_RATE_RAD = -0.5  # rad/s

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
sim = pd.read_csv("src/simulation.csv")
wp = pd.read_csv("src/waypoints.csv")

required_sim_cols = {"t", "x", "y", "psi", "delta", "v", "delta_dot"}
required_wp_cols = {"x", "y"}

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
delta_deg = sim["delta"].values * RAD2DEG
delta_dot_deg = sim["delta_dot"].values * RAD2DEG
v = sim["v"].values

wp_x = wp["x"].values
wp_y = wp["y"].values

# =========================
# Tracking error computation
# =========================
tracking_error = np.zeros(len(x))

for i in range(len(x)):
    dx = wp_x - x[i]
    dy = wp_y - y[i]
    tracking_error[i] = np.sqrt(np.min(dx**2 + dy**2))

# =========================
# Plotting
# =========================
fig, axs = plt.subplots(3, 2, figsize=(16, 14))
fig.suptitle("Controller Behavior Visualization Dashboard", fontsize=16)

# -------------------------
# 1. Path tracking (XY)
# -------------------------
ax = axs[0, 0]
ax.plot(wp_x, wp_y, "k--", linewidth=2, label="Waypoints")
ax.plot(x, y, "b", linewidth=2, label="Vehicle Path")
ax.scatter(x[0], y[0], c="green", s=80, label="Start")
ax.scatter(x[-1], y[-1], c="red", s=80, label="End")
ax.set_xlabel("x [m]")
ax.set_ylabel("y [m]")
ax.set_title("Path Tracking (XY)")
ax.axis("equal")
ax.grid(True)
ax.legend()

# -------------------------
# 2. Heading vs time
# -------------------------
ax = axs[0, 1]
ax.plot(t, psi_deg, linewidth=2)
ax.set_xlabel("Time [s]")
ax.set_ylabel("Heading ψ [deg]")
ax.set_title("Heading vs Time")
ax.grid(True)

# -------------------------
# 3. Steering angle vs time
# -------------------------
ax = axs[1, 0]
ax.plot(t, delta_deg, linewidth=2, label="Steering angle")
ax.axhline(MAX_STEERING_ANGLE_DEG, color="red", linestyle="--", label="Steering limits")
ax.axhline(MIN_STEERING_ANGLE_DEG, color="red", linestyle="--")
ax.set_xlabel("Time [s]")
ax.set_ylabel("Steering angle δ [deg]")
ax.set_title("Steering Angle vs Time")
ax.grid(True)
ax.legend()

# -------------------------
# 4. Steering rate vs time
# -------------------------
ax = axs[1, 1]
ax.plot(t, delta_dot_deg, linewidth=2, label="Steering rate")
ax.axhline(MAX_STEERING_RATE_DEG, color="red", linestyle="--", label="Steering-rate limits")
ax.axhline(MIN_STEERING_RATE_DEG, color="red", linestyle="--")
ax.set_xlabel("Time [s]")
ax.set_ylabel("Steering rate δ̇ [deg/s]")
ax.set_title("Steering Rate vs Time")
ax.grid(True)
ax.legend()

# -------------------------
# 5. Velocity vs time
# -------------------------
ax = axs[2, 0]
ax.plot(t, v, linewidth=2, label="Velocity")
ax.axhline(MAX_VELOCITY, color="red", linestyle="--", label="Velocity limits")
ax.axhline(MIN_VELOCITY, color="red", linestyle="--")
ax.set_xlabel("Time [s]")
ax.set_ylabel("Velocity [m/s]")
ax.set_title("Velocity vs Time")
ax.grid(True)
ax.legend()

# -------------------------
# 6. Tracking error vs time
# -------------------------
ax = axs[2, 1]
ax.plot(t, tracking_error, linewidth=2)
ax.set_xlabel("Time [s]")
ax.set_ylabel("Tracking Error [m]")
ax.set_title("Tracking Error vs Time")
ax.grid(True)

plt.tight_layout(rect=[0, 0, 1, 0.96])
plt.show()
