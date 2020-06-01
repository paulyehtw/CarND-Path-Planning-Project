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

struct Surrouding
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

    Surrouding surrouding;
    std::map<int, vector<vector<double>>> predictions;

    /**
   * detectClosestWaypoints picks the closest waypoints on groundtruth based on ego car's location
   * @param map groundtruth
   */
    Waypoints detectClosestWaypoints(const Waypoints &map);

    /**
   * interpolateWaypoints interpolates picked closest waypoints
   * @param waypoints closest waypoints
   */
    Waypoints interpolateWaypoints(const Waypoints &waypoints);

    /**
   * updateEgoCarState updates ego car state based on interpolated_waypoints and previous_path
   * @param interpolated_waypoints interpolated closest waypoints
   * @param previous_path previous generated path
   */
    void updateEgoCarState(const Waypoints &interpolated_waypoints,
                           const Waypoints &previous_path);

    /**
   * checkSurrounding checks relative locations of surrounding vehicles
   */
    void checkSurrounding();

    /**
   * predictTraffic predicts each detection's movement
   */
    void predictTraffic();

    /**
   * updateBehaviourList updates potential behaviours based on predictions
   */
    void updateBehaviourList();

    /**
   * calculateCost calculates cost for each behaviour
   */
    double calculateCost(vector<double> s_traj,
                         vector<double> d_traj,
                         std::map<int, vector<vector<double>>> predictions);
    /**
    * calculateTrajectory computes trajectory in Frenet coordinates
    * @param target {{s, s_dot, s_ddot}, {d, d_dot, d_ddot}}
    * @param duration duration of the trajectory
    * @return Jerk-Minimized Trajectory in Frenet coordinates
    */
    vector<vector<double>> calculateTrajectory(vector<vector<double>> target, double duration);

    /**
    * calculateTarget computes desired target in Frenet coordinates
    * @param behaviour next movement of ego car
    * @param predictions predicted movement of detections
    * @param duration duration of the trajectory
    * @return desired target in Frenet coordinates
    */
    vector<vector<double>> calculateTarget(std::string behaviour,
                                           std::map<int, vector<vector<double>>> predictions,
                                           double duration);

    /**
    * bestTarget computes the best target among behaviour list
    * @return The best target in Frenet coordinates
    */
    vector<vector<double>> bestTarget();

    /**
    * generateNewPath computes the path based on best target and interpolated_waypoints
    * @param target best target
    * @param interpolated_waypoints interpolated closest waypoints
    * @param previous_path previous generated path
    * @param next_x_vals next path x to be updated
    * @param next_y_vals next path y to be updated
    * @return The best target in Frenet coordinates
    */
    void generateNewPath(const vector<vector<double>> &target,
                         const Waypoints &interpolated_waypoints,
                         const Waypoints &previous_path,
                         vector<double> &next_x_vals,
                         vector<double> &next_y_vals);

public:
    PathPlanner(){};
    ~PathPlanner(){};

    vector<Detection> sensor_detections;

    /**
   * Initialize Initializes ego car's information
   * @param car_x x coordinate in Cartesian
   * @param car_y y coordinate in Cartesian
   * @param car_s s coordinate in Frenet
   * @param car_d d coordinate in Frenet
   * @param car_v ego car velocity
   * @param car_yaw ego car heading
   */
    void Initialize(double car_x, double car_y, double car_s, double car_d, double car_v, double car_yaw);

    /**
   * planPath is the pipeline of planning optimal paths
   * @param map groundtruth
   * @param previous_path previous generated path
   * @param next_x_vals next path x to be updated
   * @param next_y_vals next path y to be updated
   */
    void planPath(const Waypoints &map,
                  const Waypoints &previous_path,
                  std::vector<double> &next_x_vals,
                  std::vector<double> &next_y_vals);
};

#endif // PATH_PLANNER_H