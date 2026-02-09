# Import libraries for data handling, numerical operations, and visualization
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

# =========================
# Parameters
# =========================
# Conversion factor from radians to degrees
RAD2DEG = 180.0 / np.pi
speed_factor = 3  # Animation speedup factor
dt = 0.05  # Simulation timestep from specs.dt in seconds

# =========================
# Load data
# =========================
sim = pd.read_csv("results/Stanley_trajectory1.csv")  # t,x_ref,y_ref,x,y,psi,delta,v,delta_dot
wp = sim[["x_ref", "y_ref"]].drop_duplicates().reset_index(drop=True)

# Extract simulation data
t = sim["t"].values
x = sim["x"].values
y = sim["y"].values
psi = sim["psi"].values
delta = sim["delta"].values
v = sim["v"].values * 3.6  # km/h

# Extract waypoint coordinates
wp_x = wp["x_ref"].values
wp_y = wp["y_ref"].values


# =========================
# Cross-Tracking Error (correct computation)
# =========================
cte = np.zeros(len(x))
for i in range(len(x)):
    dx = wp_x - x[i]
    dy = wp_y - y[i]
    # round the CTE to only 2 decimal places for better visualization
    cte[i] = np.round(np.sqrt(np.min(dx**2 + dy**2)), 2)

# =========================
# Figure setup
# =========================
fig = plt.figure(figsize=(16, 10))
gs = fig.add_gridspec(1, 1, hspace=0.3, wspace=0.3)

# -------------------------
# Main motion plot
# -------------------------
ax_main = fig.add_subplot(gs[0, :])
ax_main.set_title("Vehicle Motion Animation (CTE & Steering)", fontsize=14, fontweight='bold')

ax_main.plot(wp_x, wp_y, "k--", linewidth=2, label="Waypoints", alpha=0.7)

path_line, = ax_main.plot([], [], "b", linewidth=2, label="Vehicle Path")
vehicle_point, = ax_main.plot([], [], "bo", markersize=8)

angle_text = ax_main.text(
    0.01, 0.05, '', transform=ax_main.transAxes,
    fontsize=11, verticalalignment='bottom',
    bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
    family='monospace', fontweight='bold'
)

velocity_text = ax_main.text(0, 0, '', fontsize=10, color='purple', fontweight='bold')

ax_main.set_aspect("equal")
ax_main.grid(True, alpha=0.3)
ax_main.legend(loc='upper right')

margin = 5.0
ax_main.set_xlim(min(wp_x) - margin, max(wp_x) + margin)
ax_main.set_ylim(min(wp_y) - margin, max(wp_y) + margin)
ax_main.set_xlabel("x [m]")
ax_main.set_ylabel("y [m]")

# -------------------------
# Cross-Tracking Error plot
# -------------------------
# ax_cte = fig.add_subplot(gs[1, 0])
# ax_cte.plot(t, cte, 'k--', lw=2, alpha=0.6)

# cte_line, = ax_cte.plot([], [], 'r-', lw=2)
# cte_marker, = ax_cte.plot([], [], 'ro', markersize=8)

# ax_cte.set_xlabel("Time [s]")
# ax_cte.set_ylabel("CTE [m]")
# ax_cte.set_title("Cross-Tracking Error vs Time")
# ax_cte.grid(True)
# ax_cte.set_xlim(0, t[-1])
# ax_cte.set_ylim(0, cte.max() * 1.2)

# -------------------------
# Steering angle plot
# -------------------------
# ax_delta = fig.add_subplot(gs[1, 1])
# ax_delta.plot(t, np.degrees(delta), 'k--', lw=2, alpha=0.6)

# delta_line, = ax_delta.plot([], [], 'g-', lw=2)
# delta_marker, = ax_delta.plot([], [], 'go', markersize=8)

# ax_delta.set_xlabel("Time [s]")
# ax_delta.set_ylabel("Steering Angle [°]")
# ax_delta.set_title("Steering (δ) vs Time")
# ax_delta.grid(True)
# ax_delta.set_xlim(0, t[-1])
# ax_delta.set_ylim(np.degrees(delta).min() - 5, np.degrees(delta).max() + 5)

# =========================
# Update function
# =========================
def update(frame):
    path_line.set_data(x[:frame + 1], y[:frame + 1])
    vehicle_point.set_data([x[frame]], [y[frame]])

    angle_text.set_text(
        f"Time: {t[frame]:.2f} s\n"
        f"CTE: {cte[frame]:.3f} m\n"
        f"δ (Steering): {np.degrees(delta[frame]):.2f}°"
    )

    velocity_text.set_text(f"v: {v[frame]:.2f} km/h")
    velocity_text.set_position((x[frame] + 3, y[frame] + 3))

    # cte_line.set_data(t[:frame + 1], cte[:frame + 1])
    # cte_marker.set_data([t[frame]], [cte[frame]])

    # delta_line.set_data(t[:frame + 1], np.degrees(delta[:frame + 1]))
    # delta_marker.set_data([t[frame]], [np.degrees(delta[frame])])

    return (
        path_line,
        vehicle_point,
        angle_text,
        velocity_text,
        # cte_line,
        # cte_marker,
        # delta_line,
        # delta_marker,
    )

# =========================
# Run animation
# =========================
frame_interval_ms = (dt / speed_factor) * 1000
save_fps = int(1.0 / (dt / speed_factor))

ani = FuncAnimation(
    fig,
    update,
    frames=len(t),
    interval=frame_interval_ms,
    blit=False,
    repeat=False
)
#  Show animation after saving
plt.show()

# =========================
# Save to MP4
# =========================
print("Saving animation as MP4...")
ani.save("vehicle_motion_with_velocity.mp4", 
         writer='ffmpeg', 
         fps=save_fps, 
         dpi=100,
         bitrate=1800)
print(f"✓ Animation saved at {save_fps} FPS as MP4")

#