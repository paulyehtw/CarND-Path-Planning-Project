#ifndef EGO_CAR_H
#define EGO_CAR_H

#include "car.h"
#include "path_planner.h"
#include "waypoints.h"
#include <vector>

class EgoCar
{
    Car ego_car;
    PathPlanner path_planner;

public:
    EgoCar(){};
    ~EgoCar();
    std::vector<Car> traffic;
    void planPath(Car &ego_car_state,
                  const Waypoints &map,
                  const std::vector<Car> &traffic,
                  const Waypoints &previous_path,
                  std::vector<double> &next_x_vals,
                  std::vector<double> &next_y_vals);
};

#endif // EGO_CAR_H