#include "path_planner.h"
#include "helpers.h"

std::vector<std::vector<double>> PathPlanner::detectClosestWaypoints(const Car &ego_car_state,
                                                                     const std::vector<std::vector<double>> &map)
{
    int num_waypoints = map[0].size();
    int next_waypoint_index = NextWaypoint(ego_car_state.x,
                                           ego_car_state.y,
                                           ego_car_state.yaw,
                                           map[0] /* map_waypoints_x */,
                                           map[1] /* map_waypoints_y */);

    std::vector<std::vector<double>> closest_waypoints{};

    // Loop over x, y, s, dx, dy
    for (int map_id = 0; map_id < map.size(); map_id++)
    {
        // Add closest waypoints information
        std::vector<double> waypoints{};
        for (int i = -param.kNumWaypointsBehind; i < param.kNumWaypointsAhead; i++)
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
            waypoints.push_back(map[map_id][idx]);
        }
        closest_waypoints.push_back(waypoints);
    }

    return closest_waypoints;
}
