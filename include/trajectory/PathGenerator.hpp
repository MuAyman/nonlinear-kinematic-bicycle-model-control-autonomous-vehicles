#pragma once

#include <vector>
#include <cmath>
#include <spline.h>
#include "../types.hpp"

/*
    PathGenerator
    -------------
    Responsibilities:
    - Convert discrete waypoints (x,y) into a continuous geometric path
    - Parameterize the path by cumulative distance s
    - Provide position, heading, and curvature at any s

    Non-responsibilities:
    - No vehicle state
    - No controller logic
    - No time or velocity
*/
class PathGenerator
{
public:
    // Constructor: generates waypoints and builds the path with default parameters of 100m length and 0.5m spacing
    PathGenerator(double pathLength = 100.0, double pointsSpacing = 0.5)
    {
        generateWaypoint(pathLength, pointsSpacing); // generate waypoints
        build(); // build splines from waypoints
    }

    /*
        evaluate()
        ----------
        Evaluate the path at a given progress s.

        Input:
        - s_query: path coordinate (distance-like parameter)

        Output:
        - PathPoint containing x, y, heading, curvature
    */
    PathPoint evaluate(double s_query) const;

    /*
        getPathLength()
        ---------------
        Returns the maximum value of s (end of path).
    */
    double getPathLength() const;

protected:
    /*
        generateWaypoint()
        ------------------
        Generates a set of waypoints and saves them to "waypoints.csv".

        Inputs:
        - path_length: total length of the path in meters, defult 100m
        - points_spacing: spacing between consecutive waypoints in meters, default 0.5m

        Output:
        - Populates the internal waypoints vector
    */
    void generateWaypoint(double pathLength = 100.0, double pointsSpacing = 0.5);

    /*
        build()
        -------
        Builds cubic splines x(s) and y(s) from waypoints x,y.
    */
    void build();

private:
    std::vector<Waypoint> waypoints; // generated discrete waypoints

    std::vector<double> s_; // Cumulative arc-length parameter for waypoints

    tk::spline spline_x_; // Cubic spline x(s)
    tk::spline spline_y_; // Cubic spline y(s)

    double s_end_ = 0.0; // actual computed cumulative length after build()
};
