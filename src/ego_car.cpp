#include "ego_car.h"

void EgoCar::planPath(const Car &ego_car_state,
                      const std::vector<std::vector<double>> &map,
                      std::vector<double> &next_x_vals,
                      std::vector<double> &next_y_vals)
{
    std::vector<std::vector<double>> closest_waypoints = path_planner.detectClosestWaypoints(ego_car_state, map);
    next_x_vals = closest_waypoints[0];
    next_y_vals = closest_waypoints[1];
}
