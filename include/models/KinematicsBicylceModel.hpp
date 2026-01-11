#pragma once
#include "../types.hpp"
#include <cmath>

class KinematicsBicycleModel
{
public:
    KinematicsBicycleModel(double wheelbase) : wheelbase_(wheelbase) {};

    // Advances the vehicle state by one time step dt using the nonlniear kinematic bicycle model
    states step(const states &current_state, const inputs &control_input, const double dt) const
    {
        states next_state;

        double x = current_state.x;
        double y = current_state.y;
        double heading = current_state.heading;
        double steeringAngle = current_state.steeringAngle;

        double velocity = control_input.velocity;
        double steeringRate = control_input.steeringRate;

        // Update steering angle
        steeringAngle += steeringRate * dt;

        // Nonlinear Kinematic bicycle model (Forward Euler discretization)
        next_state.x = x + velocity * cos(heading) * dt;
        next_state.y = y + velocity * sin(heading) * dt;
        next_state.heading = heading + velocity * tan(steeringAngle) / wheelbase_ * dt;
        next_state.steeringAngle = steeringAngle;

        return next_state;
    };

    // Imposes physical limits on steering angle, steering rate, and velocity
    void imposelimits(states &current_state, inputs &control_input)
    {
        limits vehicle_limits;

        // Check and limit velocity
        if (control_input.velocity > vehicle_limits.max_velocity)
            control_input.velocity = vehicle_limits.max_velocity;
        else if (control_input.velocity < vehicle_limits.min_velocity)
            control_input.velocity = vehicle_limits.min_velocity;

        // Check and limit steering rate
        if (control_input.steeringRate > vehicle_limits.max_steering_rate)
            control_input.steeringRate = vehicle_limits.max_steering_rate;
        else if (control_input.steeringRate < vehicle_limits.min_steering_rate)
            control_input.steeringRate = vehicle_limits.min_steering_rate;

        if (current_state.steeringAngle > vehicle_limits.max_steering_angle)
            current_state.steeringAngle = vehicle_limits.max_steering_angle;
        else if (current_state.steeringAngle < vehicle_limits.min_steering_angle)
            current_state.steeringAngle = vehicle_limits.min_steering_angle;
    }

private:
    double wheelbase_ = 2.5; // default wheelbase length in meters
};