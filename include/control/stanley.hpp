#include "../types.hpp"
#include <algorithm>

class StanleyController
{
public:
    StanleyController(double k = 1.0, double wheelbase = 2.5)
        : k_(k), wheelbase_(wheelbase) {}

    double computeSteeringAngle(const states &state_errors_path, double velocity)
    {
        // Extract lateral error and heading error
        double e_y = state_errors_path.y;
        // `state_errors_path.heading` is computed as (vehicle_heading - path_heading) in ReferenceManager.
        // Stanley control expects heading error = (path_heading - vehicle_heading), so invert sign here.
        double e_psi = state_errors_path.heading;

        // Compute desired steering angle using Stanley control law
        // Use a minimum effective velocity to avoid large arctan when vehicle is near zero speed
        double v_eff = std::max(velocity, vehicle_limits.abs_min_velocity);
        double steering_angle = e_psi + std::atan2(k_ * e_y, v_eff);
        steering_angle = std::clamp(steering_angle, vehicle_limits.min_steering_angle, vehicle_limits.max_steering_angle);

        return steering_angle;
    }

private:
    double k_ = 1.0; // control gain with defualt value of 1.0
    double wheelbase_;
    limits vehicle_limits;
};