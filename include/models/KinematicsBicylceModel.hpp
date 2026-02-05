#pragma once
#include "../types.hpp"

class KinematicsBicycleModel
{
public:
    KinematicsBicycleModel() = default;

    // Advances the vehicle state by one time step dt using the nonlniear kinematic bicycle model
    states step(const states &current_state, const inputs &control_input) const
    {
        states next_state;

        double x = current_state.x;
        double y = current_state.y;
        double heading = current_state.heading;
        double steeringAngle = current_state.steeringAngle;

        double velocity = control_input.velocity;
        double steeringRate = control_input.steeringRate;

        // Update steering angle
        steeringAngle += steeringRate * specs.dt;

        // Nonlinear Kinematic bicycle model (Forward Euler discretization)
        next_state.x = x + velocity * cos(heading) * specs.dt;
        next_state.y = y + velocity * sin(heading) * specs.dt;
        next_state.heading = heading + velocity * tan(steeringAngle) / specs.wheelbase * specs.dt;
        next_state.steeringAngle = steeringAngle;

        // // Kinematic bicycle model equations (Jacobian linearization + Forward Euler discretization)
        // next_state.x = x + (-sin(heading) * heading + cos(heading)) * velocity * specs.dt;
        // next_state.y = y + (cos(heading) * heading + sin(heading)) * velocity * specs.dt;
        // next_state.heading = heading + ((steeringAngle / (wheelbase_ * pow(cos(steeringAngle), 2))) + tan(steeringAngle) / wheelbase_) * velocity * specs.dt;
        // next_state.steeringAngle = steeringAngle;

        return next_state;
    };

    // Imposes physical limits on steering angle, steering rate, and velocity
    void imposelimits(states &current_state, inputs &control_input) const
    {
        // Check and limit velocity
        if (control_input.velocity > limits.max_velocity)
            control_input.velocity = limits.max_velocity;
        else if (control_input.velocity < limits.min_velocity)
            control_input.velocity = limits.min_velocity;

        // Check and limit steering rate
        if (control_input.steeringRate > limits.max_steering_rate)
            control_input.steeringRate = limits.max_steering_rate;
        else if (control_input.steeringRate < limits.min_steering_rate)
            control_input.steeringRate = limits.min_steering_rate;

        if (current_state.steeringAngle > limits.max_steering_angle)
            current_state.steeringAngle = limits.max_steering_angle;
        else if (current_state.steeringAngle < limits.min_steering_angle)
            current_state.steeringAngle = limits.min_steering_angle;

        // Avoid very small values of velocity and steering rate at the end of path
        // Prevent that abs min value to be assigned at the beginning of the motion
        // if (std::abs(current_state.x) > 0.1 || std::abs(current_state.y) > 0.1)
        // {
        //     // Set to zero if below absolute minimum thresholds
        //     if (std::abs(control_input.velocity) < limits.abs_min_velocity)
        //         control_input.velocity = 0.0;
        //     // Set to zero if below absolute minimum thresholds
        //     if (std::abs(control_input.steeringRate) < limits.abs_min_steering_rate)
        //         control_input.steeringRate = 0.0;
        // }
    }

private:
    vehicleLimits limits; // vehicle limits
    vehicleSpecs specs;          // vehicle specs
};