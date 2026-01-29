#pragma once
#include <cmath>

class PIDController
{
public:
    // Constructor to initialize PID gains
    PIDController(double kp = 1.0, double ki = 0.0, double kd = 0.0, double dt = 0.05)
        : kp_(kp), ki_(ki), kd_(kd), dt_(dt), prev_error_(0.0), integral_(0.0) {}

    // Compute control output based on error
    double computeControl(double error)
    {
        // Proportional term
        double p_term = kp_ * error;

        // Integral term
        integral_ += error * dt_;
        double i_term = ki_ * integral_;

        // Derivative term
        double derivative = (error - prev_error_) / dt_;
        double d_term = kd_ * derivative;

        // Update previous error
        prev_error_ = error;

        // Total output
        return p_term + i_term + d_term;
    }

    // Reset the controller (e.g., when starting a new maneuver)
    void reset()
    {
        prev_error_ = 0.0;
        integral_ = 0.0;
    }

private:
    double kp_;         // Proportional gain
    double ki_;         // Integral gain
    double kd_;         // Derivative gain
    double dt_;         // Time step
    double prev_error_; // Previous error for derivative
    double integral_;   // Integral accumulation
};