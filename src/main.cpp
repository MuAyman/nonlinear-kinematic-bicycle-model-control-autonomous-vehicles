#include <fstream>
#include <iostream>
#include "../include/types.hpp"
#include "../include/models/KinematicsBicylceModel.hpp"
#include "../include/trajectory/ReferenceManager.hpp"
#include "../include/control/pure_pursuit.hpp"
#include "../include/control/p_controller.hpp"

int main()
{
    // Simulation parameters
    double wheelbase = 2.5;                   // wheelbase in meters
    limits vehicle_limits;                    // vehicle limits
    states current_state{0.0, 0.0, 0.0, 0.0}; // x, y, heading, steeringAngle
    inputs control_input{0.0, 0.0};           // velocity, steeringRate
    double max_velocity = 15.0;               // desired cruising speed in m/s
    double dt = 0.05;                         // time step in seconds
    double ds = max_velocity * dt;            // path increment for reference manager

    // Path parameters
    double pathLength = 500.0;  // path length in meters
    double pointsSpacing = 0.5; // spacing between waypoints in meters

    // Controller parameters
    double lookahead_distance = 0.5; // lookahead distance in meters
    double kp_steering = 25;         // proportional gain for steering rate - higher for more aggressive steering
    double kp_velocity = 10;         // proportional gain for velocity - higher for faster velocity response

    // Vehicle model
    KinematicsBicycleModel model(wheelbase);

    // Reference manager for path following
    ReferenceManager ref_manager(pathLength, pointsSpacing, wheelbase, ds);

    // Total simulation time based on path length and max velocity. Adding extra 25% time to ensure full stop at the end.
    double pathLen = ref_manager.getPathLength();
    double total_time = 1.5 * static_cast<int>(pathLen / max_velocity / dt);

    // Pure Pursuit controller for steering angle
    PurePursuitController pp_controller(lookahead_distance, wheelbase);

    // Proportional controller for steeringRate & velocity
    PController p_controller_steering(kp_steering);
    PController p_controller_velocity(kp_velocity);

    // Open file to save simulation data for analysis
    std::ofstream file("visualization/simulation.csv");
    file << "t,x,y,psi,delta,v,delta_dot\n";

    for (int i = 0; i < total_time; ++i)
    {
        // Get lookahead point in global frame at specified distance ahead on the path
        states state_errors_global = ref_manager.calculateErrorGlobalFrame(current_state, lookahead_distance);

        // Transform error to vehicle frame
        states state_errors_vehicle = ref_manager.errorGlobaltoVehicleFrame(state_errors_global, current_state);

        // Create Waypoint for Pure Pursuit controller
        Waypoint XYErrorVehicleFrame{state_errors_vehicle.x, state_errors_vehicle.y};

        // Compute desired steering angle using Pure Pursuit to lookahead point
        double desired_steering_angle = pp_controller.computeSteeringAngle(current_state, XYErrorVehicleFrame);

        // Compute steering angle rate from P controller
        double steering_angle_error = desired_steering_angle - current_state.steeringAngle;
        control_input.steeringRate = p_controller_steering.computeControl(steering_angle_error);

        // Reduce speed based on steering angle to enhance stability
        double delta_ratio = std::abs(current_state.steeringAngle) / vehicle_limits.max_steering_angle;
        if (max_velocity <= 5.0)
            delta_ratio = 0.0; // no speed reduction for low speeds
        double desired_velocity = max_velocity * (1.0 - delta_ratio);

        // Stop vehicle if end of path is reached using the vehicle position
        if (ref_manager.isPathCompleted())
            desired_velocity = 0.0; // stop the vehicle

        // Simple P controller: ramp velocity towards desired value
        double velocity_error = desired_velocity - control_input.velocity;
        control_input.velocity += p_controller_velocity.computeControl(velocity_error) * dt;

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
