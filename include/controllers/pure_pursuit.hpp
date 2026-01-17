#include <cmath>
#include "../types.hpp"

class PurePursuitController
{
public:
    // Constructor to initialize lookahead distance
    PurePursuitController(double lookahead_distance = 0.1, double wheelbase = 2.5) // default 0.1 meters
        : lookahead_distance_(lookahead_distance), wheelbase_(wheelbase)
    {
    }

    // Compute the desired steering angle using Pure Pursuit algorithm
    double computeSteeringAngle(const states current_state, const Waypoint XYErrorVehicleFrame) const
    {
        double local_y = XYErrorVehicleFrame.y;
        double local_x = XYErrorVehicleFrame.x;

        // Distance to lookahead point
        double distance_to_target = sqrt(local_x * local_x + local_y * local_y);
        if (distance_to_target < 1e-6)
            return 0.0; // Prevent division by zero

        // Calculate curvature to the lookahead point
        double curvature = (2.0 * local_y) / (distance_to_target * distance_to_target);
        double steering_angle = atan(wheelbase_ * curvature); // steering angle in radians

        return steering_angle;
    }

    // // Getter for lookahead distance
    // double getLookaheadDistance() const
    // {
    //     return lookahead_distance_;
    // }

private:
    double lookahead_distance_; // Lookahead distance in meters
    double wheelbase_;          // Vehicle wheelbase in meters
};