import pandas as pd
import matplotlib.pyplot as plt


# Load CSV
data = pd.read_csv("src\simulation.csv")

t = data["t"]
x = data["x"]
y = data["y"]
delta = data["delta"]
v = data["v"]

# --- Plot X vs Y ---
plt.figure()
plt.plot(x, y)
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.title("Trajectory (x vs y)")
plt.axis("equal")
plt.grid(True)

# # --- Plot steering angle vs time ---
# plt.figure()
# plt.plot(t, delta)
# plt.xlabel("time [s]")
# plt.ylabel("steering angle δ [rad]")
# plt.title("Steering Angle vs Time")
# plt.grid(True)

# # --- Plot velocity vs time ---
# plt.figure()
# plt.plot(t, v)
# plt.xlabel("time [s]")
# plt.ylabel("velocity v [m/s]")
# plt.title("Velocity vs Time")
# plt.grid(True)

plt.show()
