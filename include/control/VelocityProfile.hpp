#include <iostream>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include "../types.hpp"

class VelocityProfile
{
public:
    // Update velocity using a trapezoidal profile with constant a_max
    double trapezoidal(const double currentVel, const double distanceRemaining) const
    {
        // Guard: Stop if we are within 1mm and moving slower than 1cm/s
        if (distanceRemaining <= 1e-3 && currentVel <= 1e-2)
            return 0.0; // Goal reached

        // Local aliases for readability
        const double v_max = limits.max_velocity;
        const double a_max = limits.a_long_max;
        const double dt = specs.dt;

        double newVel = currentVel;

        // 1. Calculate Braking Distance
        // Formula: d = v^2 / (2 * a)
        double brakingDistance = (currentVel * currentVel) / (2.0 * a_max);

        // 2. Determine State and Update Velocity
        if (distanceRemaining <= brakingDistance)
        {
            // PHASE 3: RAMP-DOWN (deceleration)
            newVel -= a_max * dt;
            newVel = std::max(newVel, 0.0); // clamp to zero
        }
        else if (currentVel < v_max)
        {
            // PHASE 1: RAMP-UP (acceleration)
            newVel += a_max * dt;
            newVel = std::min(newVel, v_max); // clamp to v_max
        }
        else
        {
            // PHASE 2: HOLD (v_max)
            newVel = v_max;
        }

        return newVel;
    }

    // Update velocity using an s curve profile with varying a & constant jerk
    double SCurve(const double currentVel, const double distanceRemaining)
    {
        // Guard: Stop if we are within 1mm and moving slower than 1cm/s
        if (distanceRemaining <= 1e-3 && currentVel <= 1e-2)
        {
            currentAcc = 0.0; // reset the internal currentAcc for the next move
            return 0.0;
        }

        // Local aliases for readability based on your requirements
        const double v_max = limits.max_velocity;
        const double a_max = limits.a_long_max;
        const double j_max = limits.jerk_long_max;
        const double dt = specs.dt;

        // 1. Calculate Stopping Distance (Ds)
        // For S-Curve, Ds = (v^2 / 2a) + (v * a / 2j)
        // This accounts for the time to ramp deceleration down to zero jerk.
        double stoppingDistance = ((currentVel * currentVel) / (2.0 * a_max)) +
                                  (currentVel * a_max / (2.0 * j_max));

        double targetAcc = 0.0;

        // 2. Determine Target Acceleration Phase
        if (distanceRemaining <= stoppingDistance)
        {
            // PHASE: Deceleration (Ramp accel towards -a_max)
            targetAcc = -a_max;
        }
        else if (currentVel < v_max)
        {
            // PHASE: Acceleration (Ramp accel towards +a_max)
            targetAcc = a_max;
        }
        else
        {
            // PHASE: Cruise (Ramp accel towards 0)
            targetAcc = 0.0;
        }

        // 3. Apply Jerk-limited Acceleration
        if (currentAcc < targetAcc)
        {
            // PHASE: Acceleration (Ramp accel towards +a_max)
            currentAcc += j_max * dt;
            currentAcc = std::min(currentAcc, a_max); // clamp to a_max
        }
        else if (currentAcc > targetAcc)
        {
            // PHASE: Deceleration (Ramp accel towards -a_max)
            currentAcc -= j_max * dt;
            currentAcc = std::max(currentAcc, -a_max); // clamp to -a_max
        }

        // 4. Calculate New Velocity
        double newVel = currentVel + (currentAcc * dt);
        newVel = std::clamp(newVel, 0.0, v_max); // Ensure velocity stays within limits

        return newVel;
    }

private:
    vehicleLimits limits;
    vehicleSpecs specs;
    double currentAcc = 0.0; // Note: In a real class, this should be a member variable
};