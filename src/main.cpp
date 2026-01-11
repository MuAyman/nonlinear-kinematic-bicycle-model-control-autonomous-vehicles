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
double wheelbase = 2.5;                   // wheelbase in meters
    KinematicsBicycleModel model(wheelbase);  // vehicle model instance
    states current_state{0.0, 0.0, 0.0, 0.0}; // x, y, heading, steeringAngle
    inputs control_input{0.0, 0.0};           // velocity, steeringRate
    double dt = 0.05;                         // time step in seconds

    // reference manager to get the reference points from the path at each step
    ReferenceManager ref_manager(500,0.5,wheelbase); // generates a path with default pathLength 100m, pointSpacing 0.5m, wheelbase 2.5m

    // Pure Pursuit controller for steering angle
    PurePursuitController pp_controller(0.1, wheelbase); // lookahead distance 5.0 meters

    // Proportional controller for steeringRate & velocity
    double kp_steering = 2;                       // proportional gain for steering rate
    double kp_velocity = 0.5;                       // proportional gain for velocity
    PController p_controller_steering(kp_steering); // proportional gain for steering rate
    PController p_controller_velocity(kp_velocity); // proportional gain for velocity

    std::ofstream file("simulation.csv");    // saves the simulation data for plotting
    file << "t,x,y,psi,delta,v,delta_dot\n"; // time step in seconds

    for (int i = 0; i < 2000; ++i)
    {
        // Get lookahead point at specified distance ahead on the path
        PathPoint lookahead_point = ref_manager.getLookaheadPoint(pp_controller.getLookaheadDistance());
        
        // Compute error to lookahead point in global frame
        states lookahead_error;
        lookahead_error.x = lookahead_point.x - current_state.x;
        lookahead_error.y = lookahead_point.y - current_state.y;
        lookahead_error.heading = lookahead_point.heading - current_state.heading;
        lookahead_error.heading = atan2(sin(lookahead_error.heading), cos(lookahead_error.heading));
        lookahead_error.steeringAngle = 0;

        // Compute desired steering angle using Pure Pursuit to lookahead point
        double desired_steering_angle = pp_controller.computeSteeringAngle(current_state, lookahead_error);

        // Compute steering angle rate from P controller
        double steering_angle_error = desired_steering_angle - current_state.steeringAngle;
        control_input.steeringRate = p_controller_steering.computeControl(steering_angle_error);

        // Compute desired velocity (constant speed, decoupled from steering)
        double desired_velocity = 10.0;  // desired cruising speed in m/s
        // Simple P controller: ramp velocity towards desired value
        double velocity_error = desired_velocity - control_input.velocity;
        control_input.velocity += p_controller_velocity.computeControl(velocity_error) * dt;

        // Apply limits and step the model
        model.imposelimits(current_state, control_input);
        current_state = model.step(current_state, control_input, dt);
        
        // Advance path progress
        ref_manager.advancePathProgress();
        
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
