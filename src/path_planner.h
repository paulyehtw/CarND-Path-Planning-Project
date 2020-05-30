#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include "car.h"
#include <stdint.h>
#include <vector>

struct PlannerParameter
{
    const uint8_t kNumWaypointsBehind = 5;
    const uint8_t kNumWaypointsAhead = 5;
};

class PathPlanner
{
    PlannerParameter param;

public:
    std::vector<std::vector<double>> detectClosestWaypoints(const Car &ego_car_state,
                                                            const std::vector<std::vector<double>> &map);
    PathPlanner(){};
    ~PathPlanner();
};

#endif // PATH_PLANNER_H