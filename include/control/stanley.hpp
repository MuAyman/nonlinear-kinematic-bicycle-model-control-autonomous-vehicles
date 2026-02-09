#pragma once
#include <cmath>
#include "../types.hpp"

class StanleyController
{
public:
    // Constructor
    StanleyController() = default;

    // Compute the desired steering angle using Stanley controller
    double computeControlInput(const states &front,
                               const inputs &input,
                               const PathPoint &path)
    {
        double steeringRate = 0.0;

        double dx = front.x - path.x;
        double dy = front.y - path.y;

        double nx = sin(path.heading);
        double ny = -cos(path.heading);
        double ey = dx * nx + dy * ny;

        double heading_error = path.heading - front.heading;
        heading_error = atan2(sin(heading_error), cos(heading_error));

        double cte = atan2(k_ * ey, input.velocity + v_0);

        double steering_angle = heading_error + cte;
        steering_angle = atan2(sin(steering_angle), cos(steering_angle));

        double error_steering = steering_angle - front.steeringAngle;
        steeringRate = std::clamp(kp_steering * error_steering,
                                  -limits.max_steering_rate,
                                  limits.max_steering_rate);



        return steeringRate;
    }

private:
    double k_ = 0.5; // Stanley gain
    // double wheelbase_;
    double kp_steering = 10.0;
    double v_0 = 0.5;
    vehicleLimits limits;
    vehicleSpecs specs;
};