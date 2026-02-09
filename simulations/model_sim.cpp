// #pragma once
#include <iostream>
#include "../include/types.hpp"
#include "../include/models/KinematicsBicylceModel.hpp"

int main()
{
    // Simulation parameters
    states current_state; // x, y, heading, steeringAngle
    inputs control_input; // velocity, steeringRate
    vehicleSpecs specs;   // vehicle specs
    double total_time = 20;
    double N = 20 / specs.dt;

    // Control inputs to travel a circle of radius 10 m in 20 seconds.
    double r = 10;
    current_state.steeringAngle = atan2(specs.wheelbase, r);
    control_input.velocity = 2 * M_PI * r / total_time;

    // Vehicle model
    KinematicsBicycleModel model;
    // current_state = model.rear2front(current_state);

    // Open file
    std::ofstream file("../results/model_sim.csv");
    file << "t,x_ref,y_ref,x,y,psi,delta,v,delta_dot\n";

    for (int i = 0; i < N; ++i)
    {
        // Step Model
        model.imposelimits(current_state, control_input);
        current_state = model.step(current_state, control_input);
        // current_state = model.rear2front(current_state);

        // Save Data
        save_simulation_step(file, (i + 1) * specs.dt, 0.0, 0.0, current_state, control_input);
    }

    file.close();
    return 0;
}