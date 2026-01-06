import numpy as np
import matplotlib.pyplot as plt

class Bicycle():
    def __init__(self):
        self.xc = 0
        self.yc = 0
        self.theta = 0
        self.delta = 0
        self.beta = 0
        self.v = 0.0  # Initial velocity
        
        self.L = 2  # Wheelbase
        self.lr = 1.2  # Distance from rear axle to center of mass
        self.lf = self.L - self.lr  # Distance from front axle to center of mass
        
        self.max_w = 1.22  # Max steering rate in rad/s
        self.max_v = 20.0  # Max velocity in m/s
        self.min_v = -5.0  # Min velocity in m/s (for reverse)

        self.dt = 0.1  # Time step

    def reset(self, x=0, y=0, theta=0):
        self.xc = x
        self.yc = y
        self.theta = theta
        self.delta = 0
        self.beta = 0
        self.v = 0.0
        
    def update(self, v, w):
        # Clip the inputs to their limits
        v = np.clip(v, self.min_v, self.max_v)
        w = np.clip(w, -self.max_w, self.max_w)
        self.v = v
        
        # Update steering angle
        self.delta += w * self.dt
        
        # Calculate slip angle beta
        self.beta = np.arctan((self.lr / self.L) * np.tan(self.delta))
        
        # Update position and orientation
        self.xc += self.v * np.cos(self.theta + self.beta) * self.dt
        self.yc += self.v * np.sin(self.theta + self.beta) * self.dt
        self.theta += (self.v / self.lr) * np.sin(self.beta) * self.dt
        
    def state(self):
        return self.xc, self.yc, self.theta, self.delta, self.beta


# Simulation parameters
sample_time = 0.01  # seconds
sim_time = 30.0  # seconds
time = np.arange(0, sim_time, sample_time)

xc = np.zeros_like(time)
yc = np.zeros_like(time)
theta = np.zeros_like(time)
delta = np.zeros_like(time)
beta = np.zeros_like(time)
v = np.zeros_like(time)
w = np.zeros_like(time)

# Create vehicle instance
vehicle = Bicycle()
vehicle.dt = sample_time

# Constant velocity for figure-8
v[:] = 16/15*np.pi + 0.02 # increased velocity for better control

# Define steering angle limits
max_steering = np.arctan(2/8)  # Maximum steering angle

# Get initial state
[xc[0], yc[0], theta[0], delta[0], beta[0]] = vehicle.state()

# Define phase durations for symmetric figure-8
# Each loop should take the same time for symmetry
steps_per_loop = len(time) // 2  # Split total time into two equal loops
transition_steps = 30  # Steps for smooth transition between loops

for i in range(1, len(time)):
    if i < steps_per_loop/4 - transition_steps:
        # First 1/4 loop - right turn (clockwise)
        if vehicle.delta < max_steering:
            w[i] = vehicle.max_w
        else:
            w[i] = 0
            vehicle.delta = max_steering
            
    elif i < steps_per_loop/4 + transition_steps:
        if i == steps_per_loop/4:
            vehicle.theta = np.deg2rad(90)
        # Transition from right to left turn
        progress = (i - (steps_per_loop/4 - transition_steps)) / (2 * transition_steps)
        target_delta = max_steering * (1 - 2 * progress)  # From +max to -max
        
        if vehicle.delta > target_delta:
            w[i] = -vehicle.max_w
        elif vehicle.delta < target_delta:
            w[i] = vehicle.max_w
        else:
            w[i] = 0
            
    elif i < 5/4 * steps_per_loop - transition_steps:
        # Second loop - left turn (counter-clockwise)
        if vehicle.delta > -max_steering:
            w[i] = -vehicle.max_w
        else:
            w[i] = 0
            vehicle.delta = -max_steering
            
    elif i < 5/4 * steps_per_loop + transition_steps:
        if i == 5/4 * steps_per_loop:
            vehicle.theta = np.deg2rad(90)
        # Transition back to straight/right turn to close the loop
        progress = (i - (5/4 * steps_per_loop - transition_steps)) / transition_steps
        target_delta = -max_steering * (1 - progress)  # From -max back to 0
        
        if vehicle.delta < target_delta:
            w[i] = vehicle.max_w
        elif vehicle.delta > target_delta:
            w[i] = -vehicle.max_w
        else:
            w[i] = 0
            
    else:
        if i == 2 * steps_per_loop - 35:
            vehicle.theta = np.deg2rad(0)
        # Second loop - left turn (counter-clockwise)
        if vehicle.delta > max_steering:
            w[i] = vehicle.max_w
        else:
            v[i] += 0.04  # Slightly increase speed to help close the loop
            w[i] = 0
            vehicle.delta = max_steering
              
    vehicle.update(v[i], w[i])
    [xc[i], yc[i], theta[i], delta[i], beta[i]] = vehicle.state()

# Print start and end positions to verify closure
print(f"Start position: ({xc[0]:.3f}, {yc[0]:.3f})")
print(f"End position: ({xc[-1]:.3f}, {yc[-1]:.3f})")
print(f"Distance from start to end: {np.sqrt((xc[-1]-xc[0])**2 + (yc[-1]-yc[0])**2):.3f}")

# Plotting
plt.figure(figsize=(12, 5))

plt.subplot(1, 2, 1)
plt.axis('equal')
plt.plot(xc, yc, 'b-', linewidth=2, label='Trajectory')
plt.plot(xc[0], yc[0], 'go', markersize=8, label='Start')
plt.plot(xc[-1], yc[-1], 'ro', markersize=8, label='End')
plt.title('Figure-8 Trajectory')
plt.xlabel('X position (m)')
plt.ylabel('Y position (m)')
plt.grid(True, alpha=0.3)
plt.legend()

plt.subplot(1, 2, 2)
plt.plot(time, np.rad2deg(delta), 'r-', linewidth=2, label='Steering Angle')
plt.xlabel('Time (s)')
plt.ylabel('Steering Angle (degrees)')
plt.title('Steering Angle vs Time')
plt.grid(True, alpha=0.3)
plt.legend()

plt.tight_layout()
plt.show()

# Additional plot showing the path closure
plt.figure(figsize=(8, 8))
plt.axis('equal')
plt.plot(xc, yc, 'b-', linewidth=2, alpha=0.7)
plt.plot(xc[0], yc[0], 'go', markersize=10, label='Start (0,0)')
plt.plot(xc[-1], yc[-1], 'ro', markersize=10, label=f'End ({xc[-1]:.2f},{yc[-1]:.2f})')

# Add arrows to show direction
n_arrows = 20
arrow_indices = np.linspace(0, len(time)-1, n_arrows, dtype=int)
for idx in arrow_indices:
    if idx < len(time) - 1:
        dx = xc[idx+1] - xc[idx]
        dy = yc[idx+1] - yc[idx]
        plt.arrow(xc[idx], yc[idx], dx*10, dy*10, 
                 head_width=0.3, head_length=0.2, fc='red', ec='red', alpha=0.6)

plt.title('Figure-8 Path with Direction Arrows')
plt.xlabel('X position (m)')
plt.ylabel('Y position (m)')
plt.grid(True, alpha=0.3)
plt.legend()
plt.show()