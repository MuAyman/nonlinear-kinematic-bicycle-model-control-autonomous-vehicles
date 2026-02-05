// #pragma once
#include <iostream>
#include "../include/types.hpp"
#include "../include/models/KinematicsBicylceModel.hpp"
#include "../include/trajectory/ReferenceManager.hpp"
#include "../include/control/VelocityProfile.hpp"
#include "../include/control/pure_pursuit.hpp"

int main()
{
    // Simulation parameters
    states current_state;               // x, y, heading, steeringAngle
    inputs control_input;               // velocity, steeringRate
    vehicleLimits limits;               // vehicle limits
    vehicleSpecs specs;                 // vehicle specs
    double max_v = limits.max_velocity; // desired cruising speed in m/s
    double ds = max_v * specs.dt;       // path increment for reference manager

    // Vehicle model
    KinematicsBicycleModel model;

    std::vector<Waypoint> input_waypoints = loadWaypointsFromCSV("../trajectories/trajectory6.csv");

    // Reference manager for path following
    ReferenceManager ref_manager(input_waypoints);

    // Total simulation time
    double pathLen = ref_manager.getPathLength();
    double total_time = static_cast<int>(1.75 * pathLen / max_v / specs.dt); // adding 75% buffer time

    // Pure Pursuit controller
    PurePursuitController pp_controller;

    // Velocity profiler
    VelocityProfile v_profile;

    // Open file
    std::ofstream file("../results/PP_trajectory6.csv");
    file << "t,x_ref,y_ref,x,y,psi,delta,v,delta_dot\n";

    // Initialize simulation performance metrics
    SimulationMetrics metrics;

    // Simulation loop
    for (int i = 0; i < total_time; ++i)
    {
        states prev_state = current_state;
        // 1. Calculate Control
        // double current_ld = pp_controller.getAdaptiveLd(control_input.velocity);
        // states state_errors_global = ref_manager.calculateErrorGlobalFrame(current_state, current_ld);

        states state_errors_global = ref_manager.calculateErrorGlobalFrame(current_state, pp_controller.getLd());

        control_input.velocity = v_profile.trapezoidalProfile(control_input.velocity, ref_manager.getPathReminingDistance());

        control_input.steeringRate = pp_controller.computeControlInput(current_state, state_errors_global);

        // 2. Step Model
        model.imposelimits(current_state, control_input);
        current_state = model.step(current_state, control_input);

        // 3. Get Reference Data
        double ref_x = ref_manager.getReferencePointGlobalframe().x;
        double ref_y = ref_manager.getReferencePointGlobalframe().y;

        // 4. Update Preformance Metrics
        updateMetrics(metrics, current_state, ref_x, ref_y);

        // 5. Save Data
        save_simulation_step(file, (i + 1) * specs.dt, ref_x, ref_y, current_state, control_input);

        // 6. Update Progress
        // ds = control_input.velocity * specs.dt;
        // ref_manager.updateProgress(ds);
        ref_manager.updateProgress(current_state, prev_state);
    }

    file.close();

    // Print simulation performance metrics
    printMetrics(metrics);

    return 0;
}