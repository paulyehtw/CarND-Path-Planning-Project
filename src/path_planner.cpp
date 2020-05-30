#include "path_planner.h"
#include "helpers.h"

Waypoints PathPlanner::detectClosestWaypoints(const Car &ego_car_state,
                                              const Waypoints &map)
{
    int num_waypoints = map.waypoints_x.size();
    int next_waypoint_index = NextWaypoint(ego_car_state.x,
                                           ego_car_state.y,
                                           ego_car_state.yaw,
                                           map.waypoints_x,
                                           map.waypoints_y);

    Waypoints closest_waypoints = Waypoints();

    // Add closest waypoints information
    for (int i = -PlannerParameter::kNumWaypointsBehind; i < PlannerParameter::kNumWaypointsAhead; i++)
    {
        int idx = (next_waypoint_index + i) % num_waypoints;
        // Out of bound check
        if (idx < 0)
        {
            continue;
        }
        else if (idx > (num_waypoints - 1))
        {
            continue;
        }
        closest_waypoints.waypoints_x.push_back(map.waypoints_x[idx]);
        closest_waypoints.waypoints_y.push_back(map.waypoints_y[idx]);
        closest_waypoints.waypoints_s.push_back(map.waypoints_s[idx]);
        closest_waypoints.waypoints_dx.push_back(map.waypoints_dx[idx]);
        closest_waypoints.waypoints_dy.push_back(map.waypoints_dy[idx]);
    }
    return closest_waypoints;
}

Waypoints PathPlanner::interpolateWaypoints(const Waypoints &waypoints)
{
    // std::vector<std::vector<double>> interpolated_waypoints{};
    // double s_length = (waypoints[2][waypoints.size() - 1] - waypoints[2][0]); // use s to calcualte interpolation points
    // int num_interpolation_points = s_length / PlannerParameter::kInterpolatedWaypointsInterval;
    // return interpolated_waypoints;
}
