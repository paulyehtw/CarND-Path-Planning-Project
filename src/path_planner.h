#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include "Eigen-3.3/Eigen/Dense"
#include "detection.h"
#include "parameters.h"
#include "waypoints.h"
#include <algorithm>
#include <map>
#include <stdint.h>
#include <vector>

using std::vector;

struct TrafficStates
{
    bool car_on_left;
    bool car_on_right;
    bool car_ahead;
    void reset()
    {
        car_on_left = false;
        car_on_right = false;
        car_ahead = false;
    };
};

class PathPlanner
{
    // State of ego car
    // Cartesian coordinates
    double x;
    double y;
    double yaw;

    // Frenet coordinates
    double s;
    double s_d;
    double s_dd;
    double d;
    double d_d;
    double d_dd;

    int subpath_size;
    vector<double> s_traj_coeffs;
    vector<double> d_traj_coeffs;
    vector<std::string> behaviour_list;

    TrafficStates traffic_states;
    std::map<int, vector<vector<double>>> traffic_predictions;

    Waypoints detectClosestWaypoints(const Waypoints &map);

    Waypoints interpolateWaypoints(const Waypoints &waypoints);

    void updateEgoCarState(const Waypoints &interpolated_waypoints,
                           const Waypoints &previous_path);

    void checkSurrounding(const std::vector<Detection> &sensor_detections);

    void predictTraffic();

    void updateBehaviourList();

    double calculateCost(vector<double> s_traj,
                         vector<double> d_traj,
                         std::map<int, vector<vector<double>>> predictions);

    vector<vector<double>> calculateTrajectory(vector<vector<double>> target, double duration);

    vector<vector<double>> calculateTarget(std::string state,
                                           std::map<int, vector<vector<double>>> predictions,
                                           double duration,
                                           bool car_just_ahead);

    vector<vector<double>> generateTarget();

    void generateNewPath(const vector<vector<double>> &target,
                         const Waypoints &interpolated_waypoints,
                         const Waypoints &previous_path,
                         vector<double> &next_x_vals,
                         vector<double> &next_y_vals);

public:
    PathPlanner(){};
    ~PathPlanner(){};

    vector<Detection> sensor_detections;

    void Initialize(double car_x, double car_y, double car_s, double car_d, double car_v, double car_yaw);

    void planPath(const Waypoints &map,
                  const Waypoints &previous_path,
                  std::vector<double> &next_x_vals,
                  std::vector<double> &next_y_vals);
};

#endif // PATH_PLANNER_H