#include <fstream>
#include <iostream>
#include "../include/types.hpp"
#include "../include/models/KinematicsBicylceModel.hpp"
#include "../include/trajectory/ReferenceManager.hpp"
#include "../include/control/pure_pursuit.hpp"
#include "../include/control/p_controller.hpp"
#include "../include/control/stanley.hpp"

int main()
{
    // Simulation parameters
    double wheelbase = 2.5;                   // wheelbase in meters
    limits vehicle_limits;                    // vehicle limits
    states current_state{0.0, 0.0, 0.0, 0.0}; // x, y, heading, steeringAngle
    inputs control_input{0.0, 0.0};           // velocity, steeringRate
    double max_velocity = 10.0;               // desired cruising speed in m/s
    double dt = 0.05;                         // time step in seconds
    double ds = max_velocity * dt;            // path increment for reference manager

    // Vehicle model
    KinematicsBicycleModel model(wheelbase);

    // Reference manager for path following
    double pathLength = 500.0;  // path length in meters
    double pointsSpacing = 0.5; // spacing between waypoints in meters
    ReferenceManager ref_manager(pathLength, pointsSpacing, wheelbase, ds);

    // Total simulation time based on path length and max velocity. Adding extra 25% time to ensure full stop at the end.
    double pathLen = ref_manager.getPathLength();
    double total_time = 1.5 * static_cast<int>(pathLen / max_velocity / dt);

    // Pure Pursuit controller for steering angle
    // double lookahead_distance = 0.5; // lookahead distance in meters
    // PurePursuitController pp_controller(lookahead_distance, wheelbase);

    // Stanley Controller (not used in this main, but can be integrated similarly)
    double k_stanley = 0.3; // Stanley controller gain
    StanleyController stanley_controller(k_stanley, wheelbase);

    // Proportional controller for steeringRate & velocity
    double kp_steering = 2; // proportional gain for steering rate (reduced to avoid oscillations)
    double kp_velocity = 10; // proportional gain for velocity - higher for faster velocity response
    PController p_controller_steering(kp_steering);
    PController p_controller_velocity(kp_velocity);

    // Open file to save simulation data for analysis
    std::ofstream file("visualization/simulation.csv");
    file << "t,x,y,psi,delta,v,delta_dot\n";

    for (int i = 0; i < total_time; ++i)
    {
        // Get lookahead point in global frame at specified distance ahead on the path
        states state_errors_global = ref_manager.calculateErrorGlobalFrame(current_state);

        // Transform error to path frame
        states state_errors_path = ref_manager.errorGlobaltoPathFrame(state_errors_global, current_state);

        // Decide desired velocity before computing steering so Stanley uses the intended speed
        double desired_velocity = max_velocity;
        if (ref_manager.isPathCompleted())
            desired_velocity = 0.0; // stop the vehicle
        control_input.velocity = desired_velocity;

        // Compute desired steering angle using Stanley controller (use current desired velocity)
        double desired_steering_angle = stanley_controller.computeSteeringAngle(state_errors_path, control_input.velocity);

        // Compute desired steering angle using Pure Pursuit to lookahead point
        // Waypoint XYErrorVehicleFrame{state_errors_vehicle.x, state_errors_vehicle.y};
        // double desired_steering_angle = pp_controller.computeSteeringAngle(current_state, XYErrorVehicleFrame);

        // Compute steering angle rate from P controller
        double steering_angle_error = desired_steering_angle - current_state.steeringAngle;
        control_input.steeringRate = p_controller_steering.computeControl(steering_angle_error);

        // Reduce speed based on steering angle to enhance stability
        // double delta_ratio = std::abs(current_state.steeringAngle) / vehicle_limits.max_steering_angle;
        // if (max_velocity <= 5.0)
        //     delta_ratio = 0.0; // no speed reduction for low speeds
        // double desired_velocity = max_velocity * (1.0 - delta_ratio);
        // (velocity already set above before steering computation)

        // Apply limits and step the model
        model.imposelimits(current_state, control_input);
        current_state = model.step(current_state, control_input, dt);

        // Save data to file
        file << (i + 1) * dt
             << ", " << current_state.x
             << ", " << current_state.y
             << ", " << current_state.heading
             << ", " << current_state.steeringAngle
             << ", " << control_input.velocity
             << ", " << control_input.steeringRate
             << std::endl;

        // Adaptively update progress along the path based on current velocity
        ds = control_input.velocity * dt;
        ref_manager.updateProgress(ds);
    }

    file.close();
    return 0;
}
