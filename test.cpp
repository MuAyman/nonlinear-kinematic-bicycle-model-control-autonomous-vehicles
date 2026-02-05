#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include "spline.h"

int main() {
    // 1. Generate 10 sample points (Original Waypoints)
    // Note: These can now loop back or move vertically!
    std::vector<double> x_orig = {0, 1, 2, 2, 1, 0, 1, 3, 5, 6};
    std::vector<double> y_orig = {0, 1, 2, 4, 5, 6, 8, 9, 8, 7};

    // 2. Compute cumulative arc-length 's'
    std::vector<double> s_values;
    s_values.push_back(0.0);
    for (size_t i = 1; i < x_orig.size(); ++i) {
        double dx = x_orig[i] - x_orig[i-1];
        double dy = y_orig[i] - y_orig[i-1];
        double dist = std::sqrt(dx*dx + dy*dy);
        s_values.push_back(s_values.back() + dist);
    }

    // 3. Setup two linear splines: x(s) and y(s)
    tk::spline spline_x, spline_y;
    spline_x.set_points(s_values, x_orig, tk::spline::linear);
    spline_y.set_points(s_values, y_orig, tk::spline::linear);

    // 4. Save Original Points to CSV
    std::ofstream file_orig("original_points.csv");
    if (file_orig.is_open()) {
        file_orig << "x,y,s\n"; 
        for (size_t i = 0; i < x_orig.size(); ++i) {
            file_orig << x_orig[i] << "," << y_orig[i] << "," << s_values[i] << "\n";
        }
        file_orig.close();
        std::cout << "Saved original_points.csv" << std::endl;
    }

    // 5. Generate and Save 100 Interpolated Points to CSV
    std::ofstream file_interp("interpolated_points.csv");
    if (file_interp.is_open()) {
        file_interp << "x,y,s\n";
        
        int num_steps = 100;
        double total_length = s_values.back();
        double step_size = total_length / (num_steps - 1);

        for (int i = 0; i < num_steps; ++i) {
            double s = i * step_size;
            // Evaluate both splines at distance 's'
            double x = spline_x(s);
            double y = spline_y(s);
            file_interp << x << "," << y << "," << s << "\n";
        }
        file_interp.close();
        std::cout << "Saved interpolated_points.csv" << std::endl;
    }

    return 0;
}
