#include "ego_car.h"

void EgoCar::planPath(Car &ego_car_state,
                      const Waypoints &map,
                      const std::vector<Car> &traffic,
                      const Waypoints &previous_path,
                      std::vector<double> &next_x_vals,
                      std::vector<double> &next_y_vals)
{
    Waypoints closest_waypoints = path_planner.detectClosestWaypoints(ego_car_state, map);
    Waypoints interpolated_waypoints = path_planner.interpolateWaypoints(closest_waypoints);
    path_planner.updateCoefficients(ego_car_state, interpolated_waypoints, previous_path);
    path_planner.detectTraffic(traffic, ego_car_state);
    next_x_vals = interpolated_waypoints.x;
    next_y_vals = interpolated_waypoints.y;
}
