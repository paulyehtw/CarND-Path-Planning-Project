#ifndef EGO_CAR_H
#define EGO_CAR_H

#include "car.h"
#include "path_planner.h"
#include "waypoints.h"
#include <vector>

struct EgoCar
{
    Car ego_car;
    PathPlanner path_planner;
    EgoCar(){};
    ~EgoCar(){};
    std::vector<Car> traffic;
    void planPath(const Waypoints &map,
                  const Waypoints &previous_path,
                  std::vector<double> &next_x_vals,
                  std::vector<double> &next_y_vals)
    {
        Waypoints closest_waypoints = path_planner.detectClosestWaypoints(ego_car, map);
        Waypoints interpolated_waypoints = path_planner.interpolateWaypoints(closest_waypoints);
        path_planner.updateCoefficients(ego_car, interpolated_waypoints, previous_path);
        path_planner.predictTraffic();
        path_planner.detectTraffic(path_planner.sensor_detections, ego_car);
        path_planner.updateStates();
        vector<vector<double>> target = path_planner.generateTarget();
        path_planner.generateNewPath(target, interpolated_waypoints, previous_path, next_x_vals, next_y_vals);
    }
};

#endif // EGO_CAR_H