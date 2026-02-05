import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Load simulation data
sim = pd.read_csv("results/PP_trajectory5.csv")
# sim = pd.read_csv("results/PP_trajectory5.csv") # t,x_ref,y_ref,x,y,psi,delta,v,delta_dot


t = sim["t"].values
x_ref = sim["x_ref"].values
y_ref = sim["y_ref"].values
x = sim["x"].values
y = sim["y"].values
psi = sim["psi"].values
delta = sim["delta"].values
v = sim["v"].values

# Compute cross-tracking error
cte = np.sqrt((x - x_ref) ** 2 + (y - y_ref) ** 2)

# Compute position error components
e_x = x_ref - x  # reference x - actual x
e_y = y_ref - y  # reference y - actual y

# Analyze metrics
print("=" * 60)
print("TRACKING PERFORMANCE ANALYSIS")
print("=" * 60)
print(f"\nTotal steps: {len(t)}")
print(f"Total time: {t[-1]:.2f} seconds")
print(f"Path length: {np.sqrt((x_ref[-1] - x_ref[0])**2 + (y_ref[-1] - y_ref[0])**2):.2f} meters")

print("\n--- CROSS-TRACKING ERROR (CTE) ---")
print(f"Max CTE:    {np.max(cte):.4f} meters")
print(f"Mean CTE:   {np.mean(cte):.4f} meters")
print(f"Std Dev:    {np.std(cte):.4f} meters")
print(f"RMSE:       {np.sqrt(np.mean(cte**2)):.4f} meters")

# Compute oscillation metrics (high-frequency content in error)
# Use derivative of CTE to detect oscillations
cte_derivative = np.diff(cte) / np.diff(t)
print(f"\n--- OSCILLATION METRICS ---")
print(f"Mean |dCTE/dt|: {np.mean(np.abs(cte_derivative)):.4f} m/s (lower = smoother)")
print(f"Max |dCTE/dt|:  {np.max(np.abs(cte_derivative)):.4f} m/s")
print(f"Std Dev dCTE/dt: {np.std(cte_derivative):.4f} m/s")

# Count zero-crossings in CTE derivative (indicates oscillation frequency)
# Zero-crossing = change from increasing to decreasing error or vice versa
zero_crossings = np.sum(np.diff(np.sign(cte_derivative)) != 0)
print(f"Error zero-crossings: {zero_crossings} (lower = fewer oscillations)")
oscillation_freq = zero_crossings / (2 * t[-1])  # pairs of zero-crossings per unit time
print(f"Oscillation frequency: {oscillation_freq:.2f} Hz")

print(f"\n--- VELOCITY & STEERING ---")
print(f"Mean velocity: {np.mean(v[v > 0]):.2f} m/s")
print(f"Steering angle range: [{np.min(delta):.3f}, {np.max(delta):.3f}] rad")

# Divide trajectory into phases and analyze convergence
print(f"\n--- CONVERGENCE ANALYSIS (by phase) ---")
n_phases = 4
phase_len = len(t) // n_phases
for i in range(n_phases):
    start = i * phase_len
    end = (i + 1) * phase_len if i < n_phases - 1 else len(t)
    phase_cte_max = np.max(cte[start:end])
    phase_cte_mean = np.mean(cte[start:end])
    t_phase_mid = t[start]
    print(f"Phase {i+1} (t≈{t_phase_mid:.1f}s): Max={phase_cte_max:.4f}m, Mean={phase_cte_mean:.4f}m")

print("\n" + "=" * 60)

# Create summary plots
fig, axs = plt.subplots(2, 2, figsize=(12, 8))

# Plot 1: CTE over time
axs[0, 0].plot(t, cte, 'b-', linewidth=1.5, label='CTE')
axs[0, 0].axhline(np.mean(cte), color='r', linestyle='--', label=f'Mean: {np.mean(cte):.4f}m')
axs[0, 0].set_xlabel('Time [s]')
axs[0, 0].set_ylabel('CTE [m]')
axs[0, 0].set_title('Cross-Tracking Error vs Time')
axs[0, 0].grid(True, alpha=0.3)
axs[0, 0].legend()

# Plot 2: Trajectory
axs[0, 1].plot(x_ref, y_ref, 'k--', linewidth=2, label='Reference Path', alpha=0.7)
axs[0, 1].plot(x, y, 'b-', linewidth=1, label='Vehicle Path')
axs[0, 1].scatter(x[0], y[0], c='green', s=100, label='Start', zorder=5)
axs[0, 1].scatter(x[-1], y[-1], c='red', s=100, label='End', zorder=5)
axs[0, 1].set_xlabel('X [m]')
axs[0, 1].set_ylabel('Y [m]')
axs[0, 1].set_title('Path Tracking (XY)')
axs[0, 1].axis('equal')
axs[0, 1].grid(True, alpha=0.3)
axs[0, 1].legend()

# Plot 3: dCTE/dt (error rate)
axs[1, 0].plot(t[:-1], cte_derivative, 'g-', linewidth=1)
axs[1, 0].axhline(0, color='k', linestyle='-', alpha=0.3)
axs[1, 0].set_xlabel('Time [s]')
axs[1, 0].set_ylabel('dCTE/dt [m/s]')
axs[1, 0].set_title('Error Rate (oscillation indicator)')
axs[1, 0].grid(True, alpha=0.3)

# Plot 4: Steering angle & velocity
ax1 = axs[1, 1]
ax2 = ax1.twinx()
ax1.plot(t, np.degrees(delta), 'g-', linewidth=1.5, label='Steering δ')
ax2.plot(t, v, 'orange', linewidth=1.5, label='Velocity v')
ax1.set_xlabel('Time [s]')
ax1.set_ylabel('Steering Angle [deg]', color='g')
ax2.set_ylabel('Velocity [m/s]', color='orange')
ax1.set_title('Control Inputs vs Time')
ax1.grid(True, alpha=0.3)
ax1.tick_params(axis='y', labelcolor='g')
ax2.tick_params(axis='y', labelcolor='orange')

plt.tight_layout()
plt.savefig('results/tracking_analysis.png', dpi=100)
print("\nPlots saved to results/tracking_analysis.png")
plt.show()
