#ifndef WAYPOINTS_H
#define WAYPOINTS_H

#include <vector>

struct Waypoints
{
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> s;
    std::vector<double> dx;
    std::vector<double> dy;

    Waypoints(){};

    Waypoints(const std::vector<double> &waypoints_x,
              const std::vector<double> &waypoints_y,
              const std::vector<double> &waypoints_s,
              const std::vector<double> &waypoints_dx,
              const std::vector<double> &waypoints_dy)
    {
        x = waypoints_x;
        y = waypoints_y;
        s = waypoints_s;
        dx = waypoints_dx;
        dy = waypoints_dy;
    };

    Waypoints(const std::vector<double> &waypoints_x,
              const std::vector<double> &waypoints_y)
    {
        x = waypoints_x;
        y = waypoints_y;
    };
};

#endif // WAYPOINTS_H