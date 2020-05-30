#ifndef EGO_CAR_H
#define EGO_CAR_H

#include "car.h"
#include "path_planner.h"
#include <vector>

class EgoCar
{
    Car ego_car;
    std::vector<Car> traffic;
    PathPlanner path_planner;

public:
    EgoCar(){};
    ~EgoCar();
    void planPath(const Car &ego_car_state,
                  const std::vector<std::vector<double>> &map,
                  std::vector<double> &next_x_vals,
                  std::vector<double> &next_y_vals);
};

#endif // EGO_CAR_H