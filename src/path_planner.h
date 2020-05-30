#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include "car.h"
#include "waypoints.h"
#include <algorithm>
#include <stdint.h>
#include <vector>

struct PlannerParameter
{
    constexpr static const uint8_t kNumWaypointsBehind = 5;
    constexpr static const uint8_t kNumWaypointsAhead = 5;
    constexpr static const float kInterpolatedWaypointsInterval = 0.5F; // meter
    constexpr static const uint8_t kNumKeptPreviousPath = 25;
    constexpr static const float dt = 0.02F;       // second
    constexpr static const float kSampleDt = 0.2F; // second
    constexpr static const uint8_t kNumPathPoints = 50;
    constexpr static const uint8_t kNumSamples = 20;
    constexpr static const float kMinLeadingDistance = 8.0F; // meter
};

class PathPlanner
{
    double s;
    double s_d;
    double s_dd;
    double d;
    double d_d;
    double d_dd;
    std::vector<double> s_traj_coeffs;
    std::vector<double> d_traj_coeffs;

public:
    Waypoints detectClosestWaypoints(const Car &ego_car_state,
                                     const Waypoints &map);
    Waypoints interpolateWaypoints(const Waypoints &waypoints);

    void updateCoefficients(Car &ego_car_state,
                            const Waypoints &interpolated_waypoints,
                            const Waypoints &previous_path);

    void detectTraffic(const std::vector<Car> &traffic, const Car &ego_car_state);

    PathPlanner(){};
    ~PathPlanner();
};

#endif // PATH_PLANNER_H