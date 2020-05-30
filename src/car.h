#ifndef CAR_H
#define CAR_H

#include "path_planner.h"
#include <vector>

struct Car
{
    double x;
    double y;
    double s;
    double d;
    double v;
    double yaw;
};

class EgoCar
{
    Car ego_car;
    std::vector<Car> traffic;
    PathPlanner path_planner;
};

#endif // CAR_H