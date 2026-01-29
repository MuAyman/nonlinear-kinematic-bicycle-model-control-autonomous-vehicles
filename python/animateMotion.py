# Import libraries for data handling, numerical operations, and visualization
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# =========================
# Parameters
# =========================
# Conversion factor from radians to degrees
RAD2DEG = 180.0 / np.pi
# Speed factor to control animation playback speed (2x means twice as fast)
speed_factor = 3  # 2x speed
# To make 3x faster, set speed_factor = 3

# =========================
# Load data
# =========================
# Load simulation data (time, position, heading, steering angle, velocity)
sim = pd.read_csv("results/PP_trajectory6.csv") # t,x_ref,y_ref,x,y,psi,delta,v,delta_dot
wp = sim[["x_ref", "y_ref"]].drop_duplicates().reset_index(drop=True)

# required_sim_cols = {"t", "x", "y", "psi", "delta", "v", "delta_dot"}
# required_wp_cols = {"x_ref", "y_ref"}
# # Load waypoints data for path reference
# # wp = pd.read_csv("src/visualization/waypoints.csv")

# Extract simulation data columns into numpy arrays for faster access
t = sim["t"].values  # Time array (s)
x = sim["x"].values  # X position of vehicle (m)
y = sim["y"].values  # Y position of vehicle (m)
psi = sim["psi"].values  # Heading angle of vehicle (in radians)
delta = sim["delta"].values  # Steering angle of vehicle (in radians)
v = sim["v"].values*3.6  # vehicle velocity (km/h)

# Extract waypoint coordinates
wp_x = wp["x_ref"].values  # X coordinates of waypoints
wp_y = wp["y_ref"].values  # Y coordinates of waypoints

# =========================
# Figure setup
# =========================
# Create a 16x10 figure with GridSpec layout
fig = plt.figure(figsize=(16, 10))
# 2 rows, 2 columns with some spacing between subplots
gs = fig.add_gridspec(2, 2, hspace=0.3, wspace=0.3)

# Main motion plot (spans both columns of first row)
ax_main = fig.add_subplot(gs[0, :])
ax_main.set_title("Vehicle Motion Animation (Heading & Steering values)", fontsize=14, fontweight='bold')
# Plot the reference waypoints as a dashed line
ax_main.plot(wp_x, wp_y, "k--", linewidth=2, label="Waypoints", alpha=0.7)

# Initialize line for vehicle path (will be updated each frame)
path_line, = ax_main.plot([], [], "b", linewidth=2, label="Vehicle Path")
# Initialize marker for current vehicle position
vehicle_point, = ax_main.plot([], [], "bo", markersize=8)

# Text box to display heading and steering angle information (positioned at bottom left)
angle_text = ax_main.text(0.01, 0.05, '', transform=ax_main.transAxes,
                          fontsize=11, verticalalignment='bottom',
                          bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
                          family='monospace', fontweight='bold')

# Text to display velocity near the vehicle position
velocity_text = ax_main.text(0, 0, '', fontsize=10, color='purple', fontweight='bold')

# Set equal aspect ratio and add grid
ax_main.set_aspect("equal")
ax_main.grid(True, alpha=0.3)
ax_main.legend(loc='upper right')
# Add margins to the plot limits
margin = 5.0
ax_main.set_xlim(min(wp_x)-margin, max(wp_x)+margin)
ax_main.set_ylim(min(wp_y)-margin, max(wp_y)+margin)
ax_main.set_xlabel("x [m]")
ax_main.set_ylabel("y [m]")

# Heading angle plot (bottom left subplot)
ax_psi = fig.add_subplot(gs[1, 0])
# Plot heading angle vs time
ax_psi.plot(t, np.degrees(psi), 'r-', lw=2)
# Marker to show current time position on the heading plot
psi_marker, = ax_psi.plot([], [], 'ro', markersize=8)
ax_psi.set_xlabel("Time [s]")
ax_psi.set_ylabel("Heading [°]")
ax_psi.set_title("Heading (ψ) vs Time")
ax_psi.grid(True)
ax_psi.set_xlim(0, t[-1])
ax_psi.set_ylim(np.degrees(psi).min()-5, np.degrees(psi).max()+5)

# Steering angle plot (bottom right subplot)
ax_delta = fig.add_subplot(gs[1, 1])
# Plot steering angle vs time
ax_delta.plot(t, np.degrees(delta), 'g-', lw=2)
# Marker to show current time position on the steering plot
delta_marker, = ax_delta.plot([], [], 'go', markersize=8)
ax_delta.set_xlabel("Time [s]")
ax_delta.set_ylabel("Steering Angle [°]")
ax_delta.set_title("Steering (δ) vs Time")
ax_delta.grid(True)
ax_delta.set_xlim(0, t[-1])
ax_delta.set_ylim(np.degrees(delta).min()-5, np.degrees(delta).max()+5)

# =========================
# Update function
# =========================
# This function is called for each frame of the animation
def update(frame):
    # Update the vehicle path line with all positions up to current frame
    path_line.set_data(x[:frame+1], y[:frame+1])
    # Update the vehicle position marker
    vehicle_point.set_data([x[frame]], [y[frame]])

    # Update the text box with current time, heading angle, and steering angle
    angle_text.set_text(
        f"Time: {t[frame]:.2f} s\n"
        f"ψ (Heading): {np.degrees(psi[frame]):.2f}°\n"
        f"δ (Steering): {np.degrees(delta[frame]):.2f}°"
    )

    # Update velocity text and position it near the vehicle
    velocity_text.set_text(f"v: {v[frame]:.2f} km/h")
    velocity_text.set_position((x[frame] + 3, y[frame] + 3))  # offset slightly above/right of vehicle

    # Update the marker position on the heading angle plot
    psi_marker.set_data([t[frame]], [np.degrees(psi[frame])])
    # Update the marker position on the steering angle plot
    delta_marker.set_data([t[frame]], [np.degrees(delta[frame])])

    # Return all artists that need to be updated (for blitting optimization)
    return path_line, vehicle_point, angle_text, velocity_text, psi_marker, delta_marker

# =========================
# Run animation
# =========================
# Create animation that updates frames with the specified interval
ani = FuncAnimation(
    fig,  # Figure to animate
    update,  # Update function called each frame
    frames=len(t),  # Total number of frames (based on simulation data length)
    interval=1,  # Calculate interval to achieve desired playback speed
    blit=False,  # Don't use blitting (safer but slightly slower)
    repeat=False  # Don't repeat animation after completion
)

# Display the animation
plt.show()

# =========================
# Save to GIF
# =========================
# Save the animation as an animated GIF file
# ani.save("vehicle_motion_with_velocity.gif", writer='pillow', fps=20)
# print("Animation saved as vehicle_motion_with_velocity.gif")
