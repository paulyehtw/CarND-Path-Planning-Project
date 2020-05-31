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
    int subpath_size = path_planner.updateCoefficients(ego_car_state, interpolated_waypoints, previous_path);
    path_planner.predictTraffic(traffic, subpath_size);
    path_planner.updateStates();
    vector<vector<double>> target = path_planner.generateTarget(traffic, ego_car_state, subpath_size);
    path_planner.generateNewPath(target, interpolated_waypoints, previous_path, subpath_size, next_x_vals, next_y_vals);
}
