#include <iostream>
#include <fstream>
#include <cmath>

struct State {
    double x;
    double y;
    double psi;
    double delta;
};

struct Input {
    double v;
    double delta_dot;
};

int main() {
    const double L  = 2.5;    // wheelbase [m]
    const double Ts = 0.01;   // sampling time [s]
    const int N = 2000;

    // Initial state
    State x{0.0, 0.0, 0.0, 0.2};



    std::ofstream file("simulation.csv");
    file << "t,x,y,psi,delta,v,delta_dot\n";

    for (int k = 0; k < N; ++k) {
        double t = k * Ts;
    // Constant input
    Input u{5.0, 0.2*sin(0.5*t)};
        // Save data
        file << t << ","
             << x.x << ","
             << x.y << ","
             << x.psi << ","
             << x.delta << ","
             << u.v << ","
             << u.delta_dot << "\n";

        // ---- Discrete Jacobians (Euler) ----
        double Ad[4][4] = {
            {1.0, 0.0, -Ts * u.v * std::sin(x.psi), 0.0},
            {0.0, 1.0,  Ts * u.v * std::cos(x.psi), 0.0},
            {0.0, 0.0,  1.0, Ts * u.v / L / (std::cos(x.delta) * std::cos(x.delta))},
            {0.0, 0.0,  0.0, 1.0}
        };

        double Bd[4][2] = {
            {Ts * std::cos(x.psi), 0.0},
            {Ts * std::sin(x.psi), 0.0},
            {Ts * std::tan(x.delta) / L, 0.0},
            {0.0, Ts}
        };

        // ---- Linearized discrete update ----
        State x_next;
        x_next.x =
            Ad[0][0]*x.x + Ad[0][1]*x.y + Ad[0][2]*x.psi + Ad[0][3]*x.delta +
            Bd[0][0]*u.v + Bd[0][1]*u.delta_dot;

        x_next.y =
            Ad[1][0]*x.x + Ad[1][1]*x.y + Ad[1][2]*x.psi + Ad[1][3]*x.delta +
            Bd[1][0]*u.v + Bd[1][1]*u.delta_dot;

        x_next.psi =
            Ad[2][0]*x.x + Ad[2][1]*x.y + Ad[2][2]*x.psi + Ad[2][3]*x.delta +
            Bd[2][0]*u.v + Bd[2][1]*u.delta_dot;

        x_next.delta =
            Ad[3][0]*x.x + Ad[3][1]*x.y + Ad[3][2]*x.psi + Ad[3][3]*x.delta +
            Bd[3][0]*u.v + Bd[3][1]*u.delta_dot;

        x = x_next;
    }

    file.close();
    std::cout << "Saved simulation.csv\n";
    return 0;
}
