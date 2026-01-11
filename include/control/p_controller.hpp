#include <cmath>





class PController
{
public:
    // Constructor to initialize proportional gain
    PController(double kp = 1.0) : kp_(kp) {}

    /*
        computeControl()
        ----------------
        Computes the control action using a proportional controller.

        Inputs:
        - error: the current error value

        Output:
        - control_action: the computed control action
    */
    double computeControl(double error) const
    {
        return kp_ * error;
    }


private:
    double kp_; // Proportional gain

};