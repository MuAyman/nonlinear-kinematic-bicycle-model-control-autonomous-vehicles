# 🚗 Nonlinear Vehicle Trajectory Tracking Controllers (C++)

## 📌 Project Overview

This repository implements a **nonlinear kinematic bicycle model** for ground vehicle motion and evaluates multiple **trajectory tracking controllers** commonly used in autonomous and electric vehicle applications.

The project focuses on **practical control design**, clear software structure, and progressive controller complexity — starting from classical geometric controllers and moving toward **optimization-based control**.

Implemented and evaluated controllers include:
- Pure Pursuit
- Stanley Controller
- PID-based control
- Model Predictive Control (MPC)
- Nonlinear Model Predictive Control (NMPC)

Controllers are developed in **C++**, simulated on multiple path geometries, and compared in terms of:
- Tracking accuracy
- Sensitivity to speed and curvature
- Stability
- Control smoothness

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

### Modeling Assumptions

The kinematic bicycle model assumes:
- Low to moderate speeds where tire slip is negligible
- No lateral or longitudinal tire force saturation
- Flat terrain and no load transfer effects

These assumptions make the model suitable for:
- Controller prototyping
- Trajectory tracking comparison
- MPC/NMPC formulation

They are **not** intended to replace a dynamic model at high speeds.
They are **not** intended to replace a dynamic model at high speeds.

---

## 🧭 Implemented Controllers

### ✔️ Classical & Geometric Controllers

- **Pure Pursuit**
  - Geometric path tracking using lookahead points
  - Coupled with steering-rate control

- **PID Controller** *(in progress)*
  - General-purpose PID controller for various control tasks
  - Configurable gains and time step

- **Stanley Controller** *(in progress)*
  - Uses heading error and lateral path error
  - Classic Stanley method adapted for steering-rate control

### 🔄 Optimization-Based Controllers

- **MPC (Linear / Decoupled)** *(in progress)*
  - Predictive control with constraints
  - Used as an intermediate step toward NMPC

- **NMPC (Nonlinear MPC)** *(planned / partial)*
  - Full nonlinear vehicle model
  - Simultaneous optimization of velocity and steering
  - Constraint-aware and curvature-adaptive behavior


**Note:** Controllers are intentionally decoupled from trajectory representation, allowing the same controller to operate on spline-based or discrete references.

---
## 🔀 Trajectory Design Strategy

Pre-computed trajectories are designed with increasing difficulty:
- Straight and mild curves
- High-curvature turns
- Oscillatory paths (e.g., sine waves, figure-8)

This progression stresses:
- Lookahead sensitivity
- Steering saturation
- Velocity–curvature coupling
---
## 📁 Repository Structure (Current)

```text
.
├── CMakeLists.txt                 # Build configuration
├── README.md
├── include/
│   ├── models/
│   │   └── KinematicsBicycleModel.hpp
│   ├── control/
│   │   ├── pure_pursuit.hpp       # Pure Pursuit controller
│   │   ├── stanley.hpp            # Stanley controller (new)
│   │   ├── pid_controller.hpp     # PID controller (new)
│   │   └── ...
│   ├── trajectory/
│   │   ├── PathGenerator.hpp      # Path generation with splines
│   │   └── ReferenceManager.hpp   # Reference trajectory management
│   └── types.hpp                  # Common data structures & some helper functions
├── simulations/
│   ├── pure_pursuit_sim.cpp       # Main simulation with Pure Pursuit
│   └── ...
├── trajectories/                  # Pre-computed trajectories CSVs
│   ├── trajectory0.csv            # straight line
│   ├── trajectory1.csv            # gentle arc
│   ├── trajectory2.csv            # medium arc
│   ├── trajectory3.csv            # tight arc
│   ├── trajectory4.csv            # sine low freq
│   ├── trajectory5.csv            # sine high freq
│   ├── trajectory6.csv            # figure 8
│   └── ...
├── results/                       # Simulation outputs
│   ├── PP_trajectory0.csv         # pure pursuit simulating trajectory 0
│   ├── PP_trajectory1.csv         # pure pursuit simulating trajectory 1
│   └── ...
└── python/                        # Visualization scripts
```
---

## 🚧 Development Status

This project is **not final** and is under active development.

### Planned and Ongoing Work
- **PID Controller** *(in progress)*
- **Stanley Controller** *(in progress)*
- **MPC (Linear / Decoupled)** *(in progress)*
- **NMPC (Nonlinear MPC)** *(planned)*
- **Velocity planners** based on path curvature and lateral acceleration limits
- Enhanced **reference trajectory handling** (improved closest point tracking)
- **Linearization-based MPC** benchmarking
- Enhanced **solver integration** (CasADi / IPOPT / ACADOS)

### Recent Fixes and Enhancements
- ✅ Fixed spline interpolation issues by filtering duplicate waypoints
- ✅ Improved reference point tracking and path progress update
- ✅ Added PID and Stanley controllers
- ✅ Added CMake build configuration
- ✅ Enhanced path generation robustness

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
