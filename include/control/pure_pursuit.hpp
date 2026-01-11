#include <cmath>
#include "../types.hpp"

class PurePursuitController
{
public:
    // Constructor to initialize lookahead distance
    PurePursuitController(double lookahead_distance = 0.1, double wheelbase = 2.5) // default 0.1 meters
        : lookahead_distance_(lookahead_distance), wheelbase_(wheelbase) {}

    /*
        computeSteeringAngle()
        ----------------------
        Computes the required steering angle using the Pure Pursuit algorithm.

        Inputs:
        - current_x: current x position of the vehicle (Global frame)
        - current_y: current y position of the vehicle (Global frame)
        - current_heading: current heading angle of the vehicle (radians)
        - target_x: x position of the target point (Global frame)
        - target_y: y position of the target point (Global frame)

        Output:
        - steering_angle: required steering angle (radians)
    */
    double computeSteeringAngle(states current_state, states global_state_errors) const
    {
        // Calculate the vector to the target point in global frame
        double dx = global_state_errors.x;
        double dy = global_state_errors.y;

        // Rotate to vehicle frame
        double local_x = cos(current_state.heading)* dx + sin(current_state.heading) * dy;
        double local_y = -sin(current_state.heading) * dx + cos(current_state.heading) * dy;

        // Calculate the steering angle using Pure Pursuit formula
        // Distance to lookahead point
        double distance_to_target = sqrt(local_x * local_x + local_y * local_y);
        if (distance_to_target < 1e-6)
            return 0.0; // Prevent division by zero

        // Pure Pursuit: steering angle = atan(2 * L * sin(alpha) / d)
        // Simplified to: curvature = 2 * lateral_error / distance^2
        double curvature = (2.0 * local_y) / (distance_to_target * distance_to_target);
        double steering_angle = atan(wheelbase_ * curvature);  // steering angle in radians

        return steering_angle;
    }

    // Getter for lookahead distance
    double getLookaheadDistance() const
    {
        return lookahead_distance_;
    }

private:
    double lookahead_distance_; // Lookahead distance in meters
    double wheelbase_;         // Vehicle wheelbase in meters

};