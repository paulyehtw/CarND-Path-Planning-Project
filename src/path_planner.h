#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include "Eigen-3.3/Eigen/Dense"
#include "car.h"
#include "waypoints.h"
#include <algorithm>
#include <map>
#include <stdint.h>
#include <vector>

#define VEHICLE_RADIUS 1.25              // meters
#define SPEED_LIMIT 21.5 // m/s
#define VELOCITY_INCREMENT_LIMIT 0.125

// cost function weights
#define COLLISION_COST_WEIGHT 99999
#define BUFFER_COST_WEIGHT 10
#define IN_LANE_BUFFER_COST_WEIGHT 1000
#define EFFICIENCY_COST_WEIGHT 10000
#define NOT_MIDDLE_LANE_COST_WEIGHT 100

// DEPRECATED CONSTANTS
#define NUM_RANDOM_TRAJ_TO_GEN 4   // the number of perturbed trajectories to generate (for each perturbed duration)
#define NUM_TIMESTEPS_TO_PERTURB 2 // the number of timesteps, +/- target time, to perturb trajectories

// sigma values for perturbing targets
#define SIGMA_S 10.0     // s
#define SIGMA_S_DOT 3.0  // s_dot
#define SIGMA_S_DDOT 0.1 // s
#define SIGMA_D 0        // d
#define SIGMA_D_DOT 0    // d_dot
#define SIGMA_D_DDOT 0   // d_double_dot
#define SIGMA_T 0.05

#define MAX_INSTANTANEOUS_JERK 10  // m/s/s/s
#define MAX_INSTANTANEOUS_ACCEL 10 // m/s/s

#define EXPECTED_JERK_IN_ONE_SEC 2 // m/s/s
#define EXPECTED_ACC_IN_ONE_SEC 1  // m/s

#define PERCENT_V_DIFF_TO_MAKE_UP 0.5 // the percent difference between current velocity and target velocity to allow ego car to make up in a single trajectory

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
    double s;
    double s_d;
    double s_dd;
    double d;
    double d_d;
    double d_dd;
    std::vector<double> s_traj_coeffs;
    std::vector<double> d_traj_coeffs;
    std::vector<std::string> available_states;
    TrafficStates traffic_states;
    std::map<int, std::vector<std::vector<double>>> traffic_predictions;

public:
    Waypoints detectClosestWaypoints(const Car &ego_car_state,
                                     const Waypoints &map);
    Waypoints interpolateWaypoints(const Waypoints &waypoints);

    int updateCoefficients(Car &ego_car_state,
                           const Waypoints &interpolated_waypoints,
                           const Waypoints &previous_path);

    void detectTraffic(const std::vector<Car> &traffic, const Car &ego_car_state);

    void predictTraffic(const std::vector<Car> &traffic, const int &subpath_size);

    void updateStates();

    std::vector<double> get_leading_vehicle_data_for_lane(int target_lane,
                                                          std::map<int, std::vector<std::vector<double>>> predictions,
                                                          double duration);

    double calculateCost(std::vector<double> s_traj,
                                std::vector<double> d_traj,
                                std::map<int, std::vector<std::vector<double>>> predictions);

    std::vector<std::vector<double>> calculateTrajectory(std::vector<std::vector<double>> target, double duration);

    std::vector<std::vector<double>> calculateTarget(std::string state,
                                                          std::map<int, std::vector<std::vector<double>>> predictions,
                                                          double duration,
                                                          bool car_just_ahead);

    void generateTarget(const std::vector<Car> &traffic, const Car &ego_car_state, const int &subpath_size);

    PathPlanner(){};
    ~PathPlanner();
};

#endif // PATH_PLANNER_H