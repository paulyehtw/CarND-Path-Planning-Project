#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include "car.h"
#include "waypoints.h"
#include <stdint.h>
#include <vector>

struct PlannerParameter
{
    constexpr static const uint8_t kNumWaypointsBehind = 5;
    constexpr static const uint8_t kNumWaypointsAhead = 5;
    constexpr static const float kInterpolatedWaypointsInterval = 0.5; // meter
};

class PathPlanner
{
    PlannerParameter param;

public:
    Waypoints detectClosestWaypoints(const Car &ego_car_state,
                                     const Waypoints &map);
    Waypoints interpolateWaypoints(const Waypoints &waypoints);
    PathPlanner(){};
    ~PathPlanner();
};

#endif // PATH_PLANNER_H