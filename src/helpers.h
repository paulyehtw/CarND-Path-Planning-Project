#ifndef HELPERS_H
#define HELPERS_H

#include "spline.h"
#include <limits>
#include <map>
#include <math.h>
#include <string>
#include <vector>

// for convenience
using std::map;
using std::string;
using std::vector;

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2)
{
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

// Calculate closest waypoint to current x, y position
int closestWaypoint(double x, double y, const vector<double> &maps_x,
                    const vector<double> &maps_y)
{
  double closestLen = std::numeric_limits<double>::max();
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i)
  {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x, y, map_x, map_y);
    if (dist < closestLen)
    {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int nextWaypoint(double x, double y, double theta, const vector<double> &maps_x,
                 const vector<double> &maps_y)
{
  int closest_waypoint = closestWaypoint(x, y, maps_x, maps_y);

  double map_x = maps_x[closest_waypoint];
  double map_y = maps_y[closest_waypoint];

  double heading = atan2((map_y - y), (map_x - x));

  double angle = fabs(theta - heading);
  angle = std::min(2 * pi() - angle, angle);

  if (angle > pi() / 2)
  {
    ++closest_waypoint;
    if (closest_waypoint == maps_x.size())
    {
      closest_waypoint = 0;
    }
  }

  return closest_waypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y, vector<double> maps_s)
{
  int next_wp = nextWaypoint(x, y, theta, maps_x, maps_y);
  int prev_wp;
  prev_wp = next_wp - 1;
  if (next_wp == 0)
  {
    prev_wp = maps_x.size() - 1;
  }
  double n_x = maps_x[next_wp] - maps_x[prev_wp];
  double n_y = maps_y[next_wp] - maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];
  // find the projection of x onto n
  double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
  double proj_x = proj_norm * n_x;
  double proj_y = proj_norm * n_y;
  double frenet_d = distance(x_x, x_y, proj_x, proj_y);
  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000 - maps_x[prev_wp];
  double center_y = 2000 - maps_y[prev_wp];
  double centerToPos = distance(center_x, center_y, x_x, x_y);
  double centerToRef = distance(center_x, center_y, proj_x, proj_y);
  if (centerToPos <= centerToRef)
  {
    frenet_d *= -1;
  }
  // calculate s value
  double frenet_s = maps_s[0];
  for (int i = 0; i < prev_wp; i++)
  {
    frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
  }
  frenet_s += distance(0, 0, proj_x, proj_y);
  return {frenet_s, frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s,
                     const vector<double> &maps_x,
                     const vector<double> &maps_y)
{
  int prev_wp = -1;

  while (s > maps_s[prev_wp + 1] && (prev_wp < (int)(maps_s.size() - 1)))
  {
    ++prev_wp;
  }

  int wp2 = (prev_wp + 1) % maps_x.size();

  double heading = atan2((maps_y[wp2] - maps_y[prev_wp]),
                         (maps_x[wp2] - maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s - maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
  double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

  double perp_heading = heading - pi() / 2;

  double x = seg_x + d * cos(perp_heading);
  double y = seg_y + d * sin(perp_heading);

  return {x, y};
}

vector<double> interpolate(vector<double> pts_x,
                           vector<double> pts_y,
                           double interval,
                           int num_points)
{
  tk::spline s;
  s.set_points(pts_x, pts_y);
  vector<double> interpolated;
  for (int i = 0; i < num_points; i++)
  {
    interpolated.push_back(s(pts_x[0] + i * interval));
  }
  return interpolated;
}

vector<double> interpolate(vector<double> pts_x,
                           vector<double> pts_y,
                           vector<double> eval_at_x)
{
  tk::spline s;
  s.set_points(pts_x, pts_y);
  vector<double> interpolated;
  for (double x : eval_at_x)
  {
    interpolated.push_back(s(x));
  }
  return interpolated;
}

namespace TrajectoryHelper
{
  std::vector<double> getTrajCoeffs(std::vector<double> start, std::vector<double> end, double T)
  {
    /*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.

    INPUTS
    start - the vehicles start location given as a length three array
        corresponding to initial values of [s, s_dot, s_double_dot]
    end   - the desired end state for vehicle. Like "start" this is a
        length three array.
    T     - The duration, in seconds, over which this maneuver should occur.

    OUTPUT
    an array of length 6, each value corresponding to a coefficent in the polynomial
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
    */

    Eigen::MatrixXd a(3, 3);
    double T2 = T * T,
           T3 = T2 * T,
           T4 = T3 * T,
           T5 = T4 * T;
    a << T3, T4, T5,
        3 * T2, 4 * T3, 5 * T4,
        6 * T, 12 * T2, 20 * T3;
    Eigen::MatrixXd aInv = a.inverse();

    Eigen::VectorXd b(3);
    b << end[0] - (start[0] + start[1] * T + 0.5 * start[2] * T2),
        end[1] - (start[1] + start[2] * T),
        end[2] - (start[2]);
    Eigen::VectorXd alpha = aInv * b;

    std::vector<double> output = {start[0], start[1], 0.5 * start[2], alpha[0], alpha[1], alpha[2]};
    return output;
  }

  std::vector<double> leadingCarInfo(int target_lane,
                                     std::map<int, std::vector<std::vector<double>>> predictions,
                                     double duration,
                                     double s)
  {
    // returns s and s_dot for the nearest (ahead) vehicle in target lane
    // this assumes the dummy vehicle will keep its lane and velocity, it will return the end position
    // and velocity (based on difference between last two positions)
    double nearest_leading_vehicle_speed = 0, nearest_leading_vehicle_distance = 99999;
    for (auto prediction : predictions)
    {
      vector<vector<double>> pred_traj = prediction.second;
      int pred_lane = pred_traj[0][1] / 4;
      if (pred_lane == target_lane)
      {
        double start_s = pred_traj[0][0];
        double predicted_end_s = pred_traj[pred_traj.size() - 1][0];
        double next_to_last_s = pred_traj[pred_traj.size() - 2][0];
        double dt = duration / Planner::kNumSamples;
        double predicted_s_dot = (predicted_end_s - next_to_last_s) / dt;
        if (predicted_end_s < nearest_leading_vehicle_distance && start_s > s)
        {
          nearest_leading_vehicle_distance = predicted_end_s;
          nearest_leading_vehicle_speed = predicted_s_dot;
        }
      }
    }
    return {nearest_leading_vehicle_distance, nearest_leading_vehicle_speed};
  }
} // namespace TrajectoryHelper

namespace CostHelper
{
  double logistic(double x)
  {
    return 2.0 / (1 + exp(-x)) - 1.0;
  }

  double distanceToCar(vector<double> s_traj, vector<double> d_traj, vector<vector<double>> prediction)
  {
    double closest = std::numeric_limits<double>::max();
    for (int i = 0; i < Planner::kNumSamples; i++)
    {
      double current_dist = sqrt(pow(s_traj[i] - prediction[i][0], 2) + pow(d_traj[i] - prediction[i][1], 2));
      if (current_dist < closest)
      {
        closest = current_dist;
      }
    }
    return closest;
  }

  double closestDistanceToCar(vector<double> s_traj, vector<double> d_traj, map<int, vector<vector<double>>> predictions)
  {
    // Determines the nearest the vehicle comes to any other vehicle throughout a trajectory
    double closest = std::numeric_limits<double>::max();
    for (auto prediction : predictions)
    {
      double current_dist = distanceToCar(s_traj, d_traj, prediction.second);
      if (current_dist < closest)
      {
        closest = current_dist;
      }
    }
    return closest;
  }

  double distanceToCarInEgoLane(vector<double> s_traj, vector<double> d_traj, map<int, vector<vector<double>>> predictions)
  {
    // Determines the nearest the vehicle comes to any other vehicle throughout a trajectory
    double closest = std::numeric_limits<double>::max();
    for (auto prediction : predictions)
    {
      double my_final_d = d_traj[d_traj.size() - 1];
      int my_lane = my_final_d / 4;
      vector<vector<double>> pred_traj = prediction.second;
      double pred_final_d = pred_traj[pred_traj.size() - 1][1];
      int pred_lane = pred_final_d / 4;
      if (my_lane == pred_lane)
      {
        double current_dist = distanceToCar(s_traj, d_traj, prediction.second);
        if (current_dist < closest && current_dist < 120)
        {
          closest = current_dist;
        }
      }
    }
    return closest;
  }

  vector<double> velocitiesTrajectory(vector<double> traj)
  {
    // given a trajectory (a vector of positions), return the average velocity between each pair as a vector
    // also can be used to find accelerations from velocities, jerks from accelerations, etc.
    // (i.e. discrete derivatives)
    vector<double> velocities;
    for (int i = 1; i < traj.size(); i++)
    {
      velocities.push_back((traj[i] - traj[i - 1]) / Planner::kSampleDt);
    }
    return velocities;
  }

  bool checkCollision(vector<double> s_traj, vector<double> d_traj, map<int, vector<vector<double>>> predictions)
  {
    // Binary cost function which penalizes collisions.
    double distnace = closestDistanceToCar(s_traj, d_traj, predictions);
    if (distnace < Planner::kCollisionBuffer)
    {
      return true;
    }
    return false;
  }

  double leadingCarCost(vector<double> s_traj, vector<double> d_traj, map<int, vector<vector<double>>> predictions)
  {
    // Penalizes getting close to other vehicles.
    double distnace = distanceToCarInEgoLane(s_traj, d_traj, predictions);
    return logistic(Planner::kCollisionBuffer / distnace);
  }

  double efficiencyCost(vector<double> s_traj)
  {
    // Rewards high average speeds.
    vector<double> s_dot_traj = velocitiesTrajectory(s_traj);
    double final_s_dot, total = 0;

    final_s_dot = s_dot_traj[s_dot_traj.size() - 1];
    return logistic((Road::kSpeedLimit - final_s_dot) / Road::kSpeedLimit);
  }

  double noneMiddleLaneCost(vector<double> d_traj)
  {
    // penalize for deviating from middle lane (d = 6)
    double end_d = d_traj[d_traj.size() - 1];
    return logistic(pow(end_d - 6, 2));
  }

} // namespace CostHelper
#endif // HELPERS_H