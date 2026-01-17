# 🚗 Nonlinear Vehicle Trajectory Tracking Controllers (C++)

## 📌 Project Overview

This repository implements a **nonlinear kinematic bicycle model** for ground vehicle motion and evaluates multiple **trajectory tracking controllers** commonly used in autonomous and electric vehicle applications.

The project focuses on **practical control design**, clear software structure, and progressive controller complexity — starting from classical geometric controllers and moving toward **optimization-based control**.

Implemented and evaluated controllers include:
- PID-based control
- Pure Pursuit
- Stanley Controller
- Model Predictive Control (MPC)
- Nonlinear Model Predictive Control (NMPC)

Controllers are developed in **C++**, simulated on multiple path geometries, and compared in terms of:
- Tracking accuracy
- Stability
- Control smoothness
- Sensitivity to speed and curvature

⚠️ **This repository is under active development.**  
Some controllers and features are still experimental or partially implemented.

---

## 🎯 Objectives

- Implement a **nonlinear kinematic bicycle model** suitable for control design
- Compare **classical, geometric, and optimal controllers**
- Study the interaction between **velocity and steering dynamics**
- Progressively transition from heuristic control to **fully coupled NMPC**
- Build a modular, extensible C++ control framework

---

## 🧠 Vehicle Model

The system uses a **kinematic bicycle model** with the following formulation:

### State Vector
$$
\mathbf{x} = [x\; y\; \psi\; \delta]
$$

Where:  
- \(x, y\): global position  
- \(\psi\): heading angle  
- \(\delta\): steering angle  

### Control Inputs
$$
\mathbf{u} = [v\; \dot{\delta}]
$$

Where:  
- \(v\): longitudinal velocity  
- \(\dot{\delta}\): steering angle rate    

This formulation allows:
- Realistic steering actuation limits
- Smooth steering behavior
- Direct compatibility with MPC/NMPC

---

## 🧭 Implemented Controllers

### ✔️ Classical & Geometric Controllers

- **PID Controller**
  - Used for velocity and steering rate control
  - Enables smooth actuation and error correction

- **Pure Pursuit**
  - Geometric path tracking using lookahead points
  - Coupled with steering-rate control

- **Stanley Controller**
  - Uses heading error and lateral path error
  - Extended to steering-rate control formulation

### 🔄 Optimization-Based Controllers

- **MPC (Linear / Decoupled)** *(in progress)*
  - Predictive control with constraints
  - Used as an intermediate step toward NMPC

- **NMPC (Nonlinear MPC)** *(planned / partial)*
  - Full nonlinear vehicle model
  - Simultaneous optimization of velocity and steering
  - Constraint-aware and curvature-adaptive behavior

---

## 📁 Repository Structure (Current)

```text
.
│
├─ include/
│   ├─ models/
│   │   ├─ VehicleModel.hpp               # Base abstract class
│   │   ├─ KinematicBicycleNonlinear.hpp
│   │   ├─ KinematicBicycleLinearSS.hpp
│   │   └─ KinematicBicycleNonlinearSS.hpp
│   │
│   ├─ controllers/
│   │   ├─ Controller.hpp                 # Base class; includes requires_velocity_profile
│   │   ├─ PurePursuit.hpp
│   │   ├─ Stanley.hpp
│   │   ├─ PID.hpp                         # Generic PID, used for steering and velocity
│   │   ├─ LQR.hpp
│   │   ├─ MPC.hpp
│   │   └─ NLMPC.hpp
│   │
│   ├─ trajectory/
│   │   ├─ PathGenerator.hpp              # Generates smooth paths from waypoints (x, y)
│   │   └─ ReferenceManager.hpp           # Provides reference points along the path; computes errors
│   │
│   └─ types.hpp                          # Structs for State, ControlInput, VehicleLimits, WayPoints, PathPoints
│
├─ python/
│   ├─ plot_trajectory.py
│   ├─ animate_controller.py
│   └─ compare_controllers.py
│
├─ results/                               # CSV files storing controller outputs
│   ├─ PurePursuit.csv
│   ├─ Stanley.csv
│   └─ ...
│
├─ data/                                  # Input waypoint sets / path points
│   ├─ waypoints1.csv
│   └─ ...
│
└─ main.cpp                               # Simulation loop, controller selection, results logging
```
---

## 🚧 Development Status

This project is **not final** and is under active development.

### Planned and Ongoing Work
- Full **Nonlinear MPC (NMPC)** implementation with constraints  
- **Velocity planners** based on path curvature and lateral acceleration limits  
- Improved **reference trajectory handling**  
- **Linearization-based MPC** benchmarking  
- Enhanced **solver integration** (CasADi / IPOPT / ACADOS)

### What to Expect
- API changes  
- Iterative controller tuning  
- Partial or experimental implementations in some modules  

---

## 📜 License

MIT License

---

## ✍️ Notes

This repository prioritizes **clarity, control correctness, and extensibility** over short-term performance optimizations.  
Each controller is implemented with the intent of being **understandable, comparable, and replaceable**, enabling systematic progression from classical controllers to advanced optimal control methods.
