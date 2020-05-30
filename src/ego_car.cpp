#include "ego_car.h"

void EgoCar::planPath(const Car &ego_car_state,
                      const Waypoints &map,
                      std::vector<double> &next_x_vals,
                      std::vector<double> &next_y_vals)
{
    Waypoints closest_waypoints = path_planner.detectClosestWaypoints(ego_car_state, map);
    // Waypoints interpolated_waypoints = path_planner.interpolateWaypoints(closest_waypoints);
    next_x_vals = closest_waypoints.waypoints_x;
    next_y_vals = closest_waypoints.waypoints_y;
}
