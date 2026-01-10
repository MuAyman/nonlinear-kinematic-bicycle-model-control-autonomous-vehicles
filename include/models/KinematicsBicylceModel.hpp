#pragma once
#include "../types.hpp"

class KinematicsBicycleModel
{

    double wheelbase = 2.5; // default wheelbase length in meters
public:
    KinematicsBicycleModel(double wheelbase_);

    states step(const states &current_state, const inputs &control_input, double dt);
    void imposelimits(states &current_state, inputs &control_input);
};