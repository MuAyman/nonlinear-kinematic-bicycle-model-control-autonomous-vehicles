#include <cmath>

class PController
{
public:
    // Constructor to initialize proportional gain
    PController(double kp = 1.0) : kp_(kp) {}

    // Compute control output based on error
    double computeControl(double error) const
    {
        return kp_ * error;
    }

private:
    double kp_; // Proportional gain
};