#pragma once
#include <vector>
#include <cmath>
#include <spline.h>
#include "../types.hpp"
#include <iostream>
#include <fstream>

/*
    PathGenerator
    -------------
    Generates a smooth path using cubic splines from a set of waypoints.

    Features:
    - Generates waypoints along a parametric path with curves and turns.
    - Builds cubic splines x(s) and y(s) from the waypoints.
    - Evaluates position, heading, and curvature at any path parameter s.

    Usage:
    - Instantiate with desired path length and waypoint spacing.
    - Call evaluate(s_query) to get PathPoint at specified s.

    Note:
    - Cubic splines ensure continuity of position, heading, and curvature.
*/
class PathGenerator
{
public:
    // Constructor: generates waypoints and builds the path with default parameters of 100m length and 0.5m spacing
    // PathGenerator(double pathLength = 100.0)
    // {
    //     generateWaypoint(pathLength); // generate waypoints
    //     build();                                     // build splines from waypoints
    // }

    // Constructor: builds the path from provided waypoints
    PathGenerator(const std::vector<Waypoint> &input_waypoints)
        : waypoints(input_waypoints)
    {
        build(); // build splines from provided waypoints
    }

    // Evaluates x(s), y(s), heading ψ(s), and curvature κ(s) at given s_query.
    PathPoint evaluate(double s_query) const
    {
        // Clamp s to the valid path interval
        double s = std::clamp(s_query, 0.0, s_end_);

        PathPoint p;

        // Position from splines
        p.x = spline_x_(s);
        p.y = spline_y_(s);

        // First derivatives with respect to s for heading calculation
        double dx_ds = spline_x_.deriv(1, s);
        double dy_ds = spline_y_.deriv(1, s);

        // Heading ψ(s) is the angle of the tangent vector.
        p.heading = std::atan2(dy_ds, dx_ds);

        // Second derivatives with respect to s for curvature calculation
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
    double getPathLength() const
    {
        return s_end_;
    }

protected:
    // Generates waypoints along a parametric path with curves and turns.
    // void generateWaypoint(double pathLength = 100.0, double pointsSpacing = 0.5)
    // {
    //     // Output file
    //     std::ofstream file("../data/waypoints1.csv");
    //     if (!file.is_open())
    //         std::cerr << "Error opening file!" << std::endl;

    //     file << "x,y\n"; // CSV header

    //     int N = static_cast<int>(pathLength / pointsSpacing);

    //     // Generate parametric path
    //     for (int i = 0; i <= N; ++i)
    //     {
    //         double s = i * pointsSpacing; // path parameter

    //         // Generate x as a steadily increasing value
    //         double x = s;

    //         // Generate y as combination of sines to create curves and turns
    //         // Wide turn: small frequency, small amplitude
    //         // Sharp turn: higher frequency, higher amplitude
    //         double y = 50.0 * sin(0.05 * s)  // wide gentle curve
    //                    + 8.0 * sin(0.2 * s); // sharper local turn

    //         // path for testing a longtudinal & lateral controller
    //         // double y = 50.0 * sin(0.05 * s)  // wide gentle curve
    //         //            + 15.0 * sin(0.2 * s); // sharper local turn

    //         waypoints.push_back({x, y});
    //         file << x << "," << y << "\n";
    //     }
    //     file.close();
    // };

    // Builds cubic splines x(s) and y(s) from waypoints x,y.
    void build()
    {
        std::vector<double> xs;
        std::vector<double> ys;
        for (const auto &wp : waypoints)
        {
            xs.push_back(wp.x);
            ys.push_back(wp.y);
        }

        // Reset s_ before building to avoid appending if build() is called multiple times
        s_.clear();

        // Filter waypoints to ensure minimum distance for spline interpolation
        std::vector<double> filtered_xs, filtered_ys;
        filtered_xs.push_back(xs[0]);
        filtered_ys.push_back(ys[0]);
        const double min_distance = 0.1; // Minimum distance between waypoints, e.g., 10 cm

        for (size_t i = 1; i < xs.size(); ++i)
        {
            double dx = xs[i] - filtered_xs.back();
            double dy = ys[i] - filtered_ys.back();
            double dist = std::sqrt(dx * dx + dy * dy);
            if (dist >= min_distance)
            {
                filtered_xs.push_back(xs[i]);
                filtered_ys.push_back(ys[i]);
            }
        }

        // Use filtered waypoints
        xs = std::move(filtered_xs);
        ys = std::move(filtered_ys);

        // Compute cumulative arc-length parameter s for each waypoint
        // s[i] = s[i-1] + distance between waypoints i and i-1
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

        // Create cubic splines for x(s) and y(s)
        spline_x_.set_points(s_, xs);
        spline_y_.set_points(s_, ys);
    }

private:

    std::vector<Waypoint> waypoints; // generated discrete waypoints
    std::vector<double> s_;          // Cumulative arc-length parameter for waypoints

    tk::spline spline_x_; // Cubic spline x(s)
    tk::spline spline_y_; // Cubic spline y(s)

    double s_end_ = 0.0;                 // actual computed cumulative length after build()
};