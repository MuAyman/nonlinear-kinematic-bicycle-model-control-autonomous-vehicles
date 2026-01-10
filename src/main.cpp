// #pragma once
#include "../include/types.hpp"
#include "../include/models/KinematicsBicylceModel.hpp"
#include "models/KinematicsBicylceModel.cpp"
#include <iostream>
#include <fstream>

int main()
{
    KinematicsBicycleModel model(2.5);        // wheelbase of 2.5 meters
    states current_state{0.0, 0.0, 0.0, 0.245}; // x, y, heading, steeringAngle
    inputs control_input{6.28, 0};       // velocity (m/s), steeringRate (rad/s)
    double dt = 0.03;

    std::ofstream file("simulation.csv");
    file << "t,x,y,psi,delta,v,delta_dot\n"; // time step in seconds

    for (int i = 0; i < 1000; ++i)
    {
        model.imposelimits(current_state, control_input);
        current_state = model.step(current_state, control_input, dt);
        file << (i + 1) * dt
             << ", " << current_state.x
             << ", " << current_state.y
             << ", " << current_state.heading
             << ", " << current_state.steeringAngle
             << ", " << control_input.velocity
             << ", " << control_input.steeringRate
             << std::endl;
    }

    file.close();
    return 0;
}
