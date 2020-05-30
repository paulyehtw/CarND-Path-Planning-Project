#ifndef WAYPOINTS_H
#define WAYPOINTS_H

#include <vector>

struct Waypoints
{
    std::vector<double> waypoints_x;
    std::vector<double> waypoints_y;
    std::vector<double> waypoints_s;
    std::vector<double> waypoints_dx;
    std::vector<double> waypoints_dy;

    Waypoints(){};

    Waypoints(const std::vector<double> &x,
              const std::vector<double> &y,
              const std::vector<double> &s,
              const std::vector<double> &dx,
              const std::vector<double> &dy)
    {
        waypoints_x = x;
        waypoints_y = y;
        waypoints_s = s;
        waypoints_dx = dx;
        waypoints_dy = dy;
    };
};

#endif // WAYPOINTS_H