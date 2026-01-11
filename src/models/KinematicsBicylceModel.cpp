#pragma once
#include "../../include/types.hpp"
#include "../../include/models/KinematicsBicylceModel.hpp"
#include <cmath>

KinematicsBicycleModel::KinematicsBicycleModel(double L) { wheelbase = L; }

states KinematicsBicycleModel::step(const states &current_state, const inputs &control_input, double dt)
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

    // Kinematic bicycle model (standard form with Forward Euler discretization)
    // ẋ = v * cos(ψ)
    // ẏ = v * sin(ψ)
    // ψ̇ = v * tan(δ) / L
    next_state.x = x + velocity * cos(heading) * dt;
    next_state.y = y + velocity * sin(heading) * dt;
    next_state.heading = heading + velocity * tan(steeringAngle) / wheelbase * dt;
    next_state.steeringAngle = steeringAngle;

    return next_state;
};

// Imposes limits on velocity and steering rate
void KinematicsBicycleModel::imposelimits(states &current_state, inputs &control_input)
{

    // Define limits
    const double max_velocity = 30.0;      // Maximum velocity in m/s
    const double min_velocity = -10.0;     // Minimum velocity in m/s (reverse)
    const double max_steering_rate = 0.5;  // Maximum steering rate in rad/s
    const double min_steering_rate = -0.5; // Minimum steering rate in rad/s

    // Check and limit velocity
    if (control_input.velocity > max_velocity)
        control_input.velocity = max_velocity;
    else if (control_input.velocity < min_velocity)
        control_input.velocity = min_velocity;

    // Check and limit steering rate
    if (control_input.steeringRate > max_steering_rate)
        control_input.steeringRate = max_steering_rate;
    else if (control_input.steeringRate < min_steering_rate)
        control_input.steeringRate = min_steering_rate;

    // limit the steering angle & heading within physical constraints
    const double max_steering_angle = M_PI / 4;  // 45 degrees in radians
    const double min_steering_angle = -M_PI / 4; // -45 degrees in radians

    if (current_state.steeringAngle > max_steering_angle)
        current_state.steeringAngle = max_steering_angle;
    else if (current_state.steeringAngle < min_steering_angle)
        current_state.steeringAngle = min_steering_angle;
}