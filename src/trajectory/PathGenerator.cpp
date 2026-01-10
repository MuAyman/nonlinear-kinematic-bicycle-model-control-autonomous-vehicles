#include "../../include/trajectory/PathGenerator.hpp"
#include <stdexcept>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>

void PathGenerator::generateWaypoint(double path_length = 100.0, double points_spacing = 0.5)
{

    // Output file
    std::ofstream file("waypoints.csv");
    if (!file.is_open())
        std::cerr << "Error opening file!" << std::endl;

    file << "x,y\n"; // CSV header

    int N = static_cast<int>(path_length / points_spacing);

    // Generate parametric path
    for (int i = 0; i <= N; ++i)
    {
        double s = i * points_spacing; // path parameter

        // Generate x as a steadily increasing value
        double x = s;

        // Generate y as combination of sines to create curves and turns
        // Wide turn: small frequency, small amplitude
        // Sharp turn: higher frequency, higher amplitude
        double y = 5.0 * sin(0.05 * s)   // wide gentle curve
                   + 2.0 * sin(0.2 * s); // sharper local turn

        waypoints.push_back({x, y});
        file << x << "," << y << "\n";
    }
    file.close();
}

// Builds cubic splines x(s) and y(s) from waypoints x,y.
void PathGenerator::build()
{

    std::vector<double> xs;
    std::vector<double> ys;
    for (const auto &wp : waypoints)
    {
        xs.push_back(wp.x);
        ys.push_back(wp.y);
    }

    // Reset internal data
    s_.clear();

    /*
        Compute cumulative arc-length approximation.

        s[0] = 0
        s[i] = s[i-1] + sqrt( (x_i - x_{i-1})^2 + (y_i - y_{i-1})^2 )

        This parameterization makes s represent distance along the path.
    */
    s_.push_back(0.0);
    for (size_t i = 1; i < xs.size(); ++i)
    {
        double dx = xs[i] - xs[i - 1];
        double dy = ys[i] - ys[i - 1];
        double ds = std::sqrt(dx * dx + dy * dy);
        s_.push_back(s_.back() + ds);
    }

    // Store total path length
    s_end_ = s_.back();

    /*
        Build cubic splines:
        - spline_x_(s) approximates x(s)
        - spline_y_(s) approximates y(s)

        Cubic splines guarantee:
        - continuous position
        - continuous first derivative (heading)
        - continuous second derivative (curvature)
    */
    spline_x_.set_points(s_, xs);
    spline_y_.set_points(s_, ys);
}

// Evaluates x(s), y(s), heading ψ(s), and curvature κ(s).
PathPoint PathGenerator::evaluate(double s_query) const
{
    // Clamp s to the valid path interval
    double s = std::clamp(s_query, 0.0, s_end_);

    PathPoint p;

    // Position from splines
    p.x = spline_x_(s);
    p.y = spline_y_(s);

    /*
        First derivatives with respect to s:
        dx/ds, dy/ds

        These define the tangent direction of the path.
    */
    double dx_ds = spline_x_.deriv(1, s);
    double dy_ds = spline_y_.deriv(1, s);

    // Heading ψ(s) is the angle of the tangent vector.
    p.heading = std::atan2(dy_ds, dx_ds);

    /*
        Second derivatives:
        d²x/ds², d²y/ds²

        Needed to compute curvature.
    */
    double d2x_ds2 = spline_x_.deriv(2, s);
    double d2y_ds2 = spline_y_.deriv(2, s);

    /*
        Curvature formula:

                    x' y'' - y' x''
        κ(s) = -------------------------
                (x'^2 + y'^2)^(3/2)
    */
    double numerator = dx_ds * d2y_ds2 - dy_ds * d2x_ds2;
    double denominator = std::pow(dx_ds * dx_ds + dy_ds * dy_ds, 1.5);

    if (denominator < 1e-6)
        p.curvature = 0.0;
    else
        p.curvature = numerator / denominator;

    return p;
}

// Returns total length of the path (maximum s).
double PathGenerator::getPathLength() const
{
    return s_end_;
}
