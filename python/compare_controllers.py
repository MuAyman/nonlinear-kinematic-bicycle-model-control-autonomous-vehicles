import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# =========================
# Controller settings
# =========================
CTRL1_NAME = "Stanley"
CTRL2_NAME = "Pure Pursuit"

CTRL1_CSV = "results/Stanley_trajectory5.csv"
CTRL2_CSV = "results/PP_trajectory5.csv"

# =========================
# Physical limits
# =========================
MAX_VELOCITY = 15.0*3.6
MIN_VELOCITY = -5.0*3.6

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
# Load CSVs
# =========================
sim1 = pd.read_csv(CTRL1_CSV)
sim2 = pd.read_csv(CTRL2_CSV)

# Check required columns
required_cols = {"t","x_ref","y_ref","x","y","psi","delta","v","delta_dot"}
for sim in [sim1, sim2]:
    if not required_cols.issubset(sim.columns):
        raise ValueError("One of the CSVs is missing required columns")

# Sort by time
sim1 = sim1.sort_values("t").reset_index(drop=True)
sim2 = sim2.sort_values("t").reset_index(drop=True)

# Extract waypoints from first CSV
wp = sim1[["x_ref","y_ref"]].drop_duplicates().reset_index(drop=True)
wp_x = wp["x_ref"].values
wp_y = wp["y_ref"].values

# =========================
# Function to process a simulation
# =========================
def process_sim(sim):
    t = sim["t"].values
    x = sim["x"].values
    y = sim["y"].values
    psi_rad = np.unwrap(sim["psi"].values)
    psi_deg = psi_rad * RAD2DEG
    delta_deg = sim["delta"].values * RAD2DEG
    delta_dot_deg = sim["delta_dot"].values * RAD2DEG
    v = sim["v"].values

    # tracking error
    cte = np.zeros(len(x))
    for i in range(len(x)):
        dx = wp_x - x[i]
        dy = wp_y - y[i]
        cte[i] = np.round(np.sqrt(np.min(dx**2 + dy**2)), 2)

    # heading error
    if len(wp_x) > 1:
        wp_dx = np.diff(wp_x)
        wp_dy = np.diff(wp_y)
        seg_angles = np.arctan2(wp_dy, wp_dx)
        psi_ref = np.zeros(len(x))
        for i_pos in range(len(x)):
            dx = wp_x - x[i_pos]
            dy = wp_y - y[i_pos]
            idx = np.argmin(dx**2 + dy**2)
            seg_idx = min(idx, len(seg_angles)-1)
            psi_ref[i_pos] = seg_angles[seg_idx]
        heading_err_rad = np.arctan2(np.sin(psi_rad - psi_ref), np.cos(psi_rad - psi_ref))
        heading_err_deg = heading_err_rad * RAD2DEG
    else:
        heading_err_deg = np.zeros(len(x))

    return t, x, y, psi_deg, delta_deg, delta_dot_deg, v, cte, heading_err_deg

t1, x1, y1, psi_deg1, delta_deg1, delta_dot_deg1, v1, cte1, heading_err1 = process_sim(sim1)
t2, x2, y2, psi_deg2, delta_deg2, delta_dot_deg2, v2, cte2, heading_err2 = process_sim(sim2)

# =========================
# FIGURE 1: Path tracking comparison
# =========================
plt.figure(figsize=(8,8))
plt.plot(wp_x, wp_y, "k--", linewidth=2, label="Waypoints")
plt.plot(x1, y1, "b", linewidth=2, label=CTRL1_NAME)
plt.plot(x2, y2, "orange", linewidth=2, label=CTRL2_NAME)
plt.scatter(x1[0], y1[0], c="green", s=80, label="Start")
plt.scatter(x1[-1], y1[-1], c="red", s=80, label="End")
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.title("Path Tracking Comparison")
plt.axis("equal")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

# =========================
# FIGURE 2: Controller dashboard comparison
# =========================
fig, axs = plt.subplots(3, 2, figsize=(16,14))
fig.suptitle("Controller Behavior Comparison - Time Domain", fontsize=16)

# Heading
axs[0,0].plot(t1, psi_deg1, "b", linewidth=2, label=CTRL1_NAME)
axs[0,0].plot(t2, psi_deg2, "orange", linewidth=2, label=CTRL2_NAME)
axs[0,0].set_ylabel("Heading ψ [deg]")
axs[0,0].set_title("Heading vs Time")
axs[0,0].grid(True)
axs[0,0].legend()

# Steering angle
axs[0,1].plot(t1, delta_deg1, "b", linewidth=2, label=CTRL1_NAME)
axs[0,1].plot(t2, delta_deg2, "orange", linewidth=2, label=CTRL2_NAME)
axs[0,1].axhline(MAX_STEERING_ANGLE_DEG, color="red", linestyle="--", label="Steering limits")
axs[0,1].axhline(MIN_STEERING_ANGLE_DEG, color="red", linestyle="--")
axs[0,1].set_ylabel("Steering angle δ [deg]")
axs[0,1].set_title("Steering Angle vs Time")
axs[0,1].grid(True)
axs[0,1].legend()

# Steering rate
axs[1,0].plot(t1, delta_dot_deg1, "b", linewidth=2, label=CTRL1_NAME)
axs[1,0].plot(t2, delta_dot_deg2, "orange", linewidth=2, label=CTRL2_NAME)
axs[1,0].axhline(MAX_STEERING_RATE_DEG, color="red", linestyle="--", label="Steering-rate limits")
axs[1,0].axhline(MIN_STEERING_RATE_DEG, color="red", linestyle="--")
axs[1,0].set_ylabel("Steering rate δ̇ [deg/s]")
axs[1,0].set_title("Steering Rate vs Time")
axs[1,0].grid(True)
axs[1,0].legend()

# Velocity
axs[1,1].plot(t1, v1*3.6, "b", linewidth=2, label=CTRL1_NAME)
axs[1,1].plot(t2, v2*3.6, "orange", linewidth=2, label=CTRL2_NAME)
axs[1,1].axhline(MAX_VELOCITY, color="red", linestyle="--", label="Velocity limits")
axs[1,1].axhline(MIN_VELOCITY, color="red", linestyle="--")
axs[1,1].set_ylabel("Velocity [km/h]")
axs[1,1].set_title("Velocity vs Time")
axs[1,1].grid(True)
axs[1,1].legend()

# Tracking error
axs[2,0].plot(t1, cte1, "b", linewidth=2, label=CTRL1_NAME)
axs[2,0].plot(t2, cte2, "orange", linewidth=2, label=CTRL2_NAME)
axs[2,0].set_ylabel("Tracking Error [m]")
axs[2,0].set_title("Tracking Error vs Time")
axs[2,0].grid(True)
axs[2,0].legend()

# Heading error
axs[2,1].plot(t1, heading_err1, "b", linewidth=2, label=CTRL1_NAME)
axs[2,1].plot(t2, heading_err2, "orange", linewidth=2, label=CTRL2_NAME)
axs[2,1].set_xlabel("Time [s]")
axs[2,1].set_ylabel("Heading error [deg]")
axs[2,1].set_title("Heading Error vs Time")
axs[2,1].grid(True)
axs[2,1].legend()

plt.tight_layout(rect=[0, 0, 1, 0.95])
plt.show()
