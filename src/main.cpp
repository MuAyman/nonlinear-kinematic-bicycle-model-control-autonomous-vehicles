// #pragma once
#include "../include/types.hpp"
#include "../include/models/KinematicsBicylceModel.hpp"
#include "models/KinematicsBicylceModel.cpp"
#include "trajectory/PathGenerator.cpp"
#include "../include/trajectory/ReferenceManager.hpp"
#include "../include/control/pure_pursuit.hpp"
#include "../include/control/p_controller.hpp"
#include <iostream>
#include <fstream>

int main()
{
    // Simulation parameters
    double wheelbase = 2.5;                   // wheelbase in meters
    states current_state{0.0, 0.0, 0.0, 0.0}; // x, y, heading, steeringAngle
    inputs control_input{0.0, 0.0};           // velocity, steeringRate
    double desired_velocity = 10.0;           // desired cruising speed in m/s
    double dt = 0.05;                         // time step in seconds

    // Path parameters
    double pathLength = 500.0;  // path length in meters
    double pointsSpacing = 0.5; // spacing between waypoints in meters

    // Controller parameters
    double lookahead_distance = 1.0; // lookahead distance in meters
    double kp_steering = 2;          // proportional gain for steering rate
    double kp_velocity = 0.5;        // proportional gain for velocity

    // Vehicle model
    KinematicsBicycleModel model(wheelbase);

    // Reference manager for path following
    ReferenceManager ref_manager(pathLength, pointsSpacing, wheelbase);

    // Pure Pursuit controller for steering angle
    PurePursuitController pp_controller(lookahead_distance, wheelbase);

    // Proportional controller for steeringRate & velocity
    PController p_controller_steering(kp_steering);
    PController p_controller_velocity(kp_velocity);

    // Open file to save simulation data for analysis
    std::ofstream file("simulation.csv");
    file << "t,x,y,psi,delta,v,delta_dot\n";

    for (int i = 0; i < 2000; ++i)
    {
        // Get lookahead point at specified distance ahead on the path
        Waypoint XYErrorVehicleFrame = ref_manager.calculateXYErrorVehicleFrame(lookahead_distance, current_state);

        // Compute desired steering angle using Pure Pursuit to lookahead point
        double desired_steering_angle = pp_controller.computeSteeringAngle(current_state, XYErrorVehicleFrame);

        // Compute steering angle rate from P controller
        double steering_angle_error = desired_steering_angle - current_state.steeringAngle;
        control_input.steeringRate = p_controller_steering.computeControl(steering_angle_error);

        // Simple P controller: ramp velocity towards desired value
        double velocity_error = desired_velocity - control_input.velocity;
        control_input.velocity += p_controller_velocity.computeControl(velocity_error) * dt;

        // Apply limits and step the model
        model.imposelimits(current_state, control_input);
        current_state = model.step(current_state, control_input, dt);

        // Advance path progress
        // ref_manager.advancePathProgress();

        // Save data to file
        file << (i + 1) * dt
             << ", " << current_state.x
             << ", " << current_state.y
             << ", " << current_state.heading
             << ", " << current_state.steeringAngle
             << ", " << control_input.velocity
             << ", " << control_input.steeringRate
             << std::endl;
    }

    file.close();
    return 0;
}
