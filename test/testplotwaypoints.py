import pandas as pd
import matplotlib.pyplot as plt

# Load waypoints CSV
df = pd.read_csv("test/waypoints.csv")

x = df['x'].values
y = df['y'].values

# Create figure
plt.figure(figsize=(12, 6))
plt.plot(x, y, label='Path', color='blue', linewidth=2)

# Highlight start and end
plt.scatter(x[0], y[0], color='green', s=80, label='Start')
plt.scatter(x[-1], y[-1], color='red', s=80, label='End')

# Optional: mark sharp turns where curvature changes rapidly
dy = y[1:] - y[:-1]
dx = x[1:] - x[:-1]
curvature = abs(dy / dx)  # approximate slope change
threshold = 0.3  # tune to highlight sharper bends
sharp_turn_indices = [i for i, k in enumerate(curvature) if k > threshold]
plt.scatter(x[sharp_turn_indices], y[sharp_turn_indices], color='orange', s=30, label='Sharp turns')

# Labels and legend
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.title("Generated Curved Waypoints Path")
plt.grid(True)
plt.axis('equal')
plt.legend()
plt.show()
