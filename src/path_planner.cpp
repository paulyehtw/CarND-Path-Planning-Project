#include "path_planner.h"
#include "helpers.h"
#include <iostream>
#include <limits>

void PathPlanner::Initialize(double car_x, double car_y, double car_s, double car_d, double car_v, double car_yaw)
{
    x = car_x;
    y = car_y;
    s = car_s;
    d = car_d;
    s_d = car_v * 0.44704; // mph to m/s
    yaw = car_yaw;
};

Waypoints PathPlanner::detectClosestWaypoints(const Waypoints &map)
{
    int num_waypoints = map.x.size();
    int next_waypoint_index = nextWaypoint(x, y, yaw, map.x, map.y);

    Waypoints closest_waypoints = Waypoints();

    // Add closest waypoints information
    for (int i = -Planner::kNumWaypointsBehind; i < Planner::kNumWaypointsAhead; i++)
    {
        int idx = (next_waypoint_index + i) % num_waypoints;
        if (idx < 0)
        {
            // correct for wrap
            idx += num_waypoints;
        }
        // correct for wrap in s for spline interpolation (must be continuous)
        double current_s = map.s[idx];
        double base_s = map.s[next_waypoint_index];
        if (i < 0 && current_s > base_s)
        {
            current_s -= Road::kTrackLength;
        }
        if (i > 0 && current_s < base_s)
        {
            current_s += Road::kTrackLength;
        }
        closest_waypoints.x.push_back(map.x[idx]);
        closest_waypoints.y.push_back(map.y[idx]);
        closest_waypoints.s.push_back(current_s);
        closest_waypoints.dx.push_back(map.dx[idx]);
        closest_waypoints.dy.push_back(map.dy[idx]);
    }
    return closest_waypoints;
}

Waypoints PathPlanner::interpolateWaypoints(const Waypoints &waypoints)
{
    Waypoints interpolated_waypoints{};
    float interval = Planner::kInterpolatedWaypointsInterval;
    double s_length = (waypoints.s[waypoints.s.size() - 1] - waypoints.s[0]); // use s to calcualte interpolation points
    int num_interpolation_points = s_length / interval;

    // Interpolate s
    interpolated_waypoints.s.push_back(waypoints.s[0]);
    for (int i = 1; i < num_interpolation_points; i++)
    {
        interpolated_waypoints.s.push_back(waypoints.s[0] + i * interval);
    }
    // Interpolate  the rest
    interpolated_waypoints.x = interpolate(waypoints.s, waypoints.x, interval, num_interpolation_points);
    interpolated_waypoints.y = interpolate(waypoints.s, waypoints.y, interval, num_interpolation_points);
    interpolated_waypoints.dx = interpolate(waypoints.s, waypoints.dx, interval, num_interpolation_points);
    interpolated_waypoints.dy = interpolate(waypoints.s, waypoints.dy, interval, num_interpolation_points);

    return interpolated_waypoints;
}

void PathPlanner::updateEgoCarState(const Waypoints &interpolated_waypoints,
                                    const Waypoints &previous_path)
{
    double pos_s, s_dot, s_ddot;
    double pos_d, d_dot, d_ddot;
    double pos_x, pos_y, angle;

    subpath_size = std::min((int)Planner::kNumKeptPreviousPath, (int)previous_path.x.size());
    double traj_start_time = subpath_size * Planner::dt;

    // use default values if not enough previous path points
    if (subpath_size < 4)
    {
        pos_x = x;
        pos_y = y;
        angle = deg2rad(yaw);
        pos_s = s;
        pos_d = d;
        s_dot = s_d;
        d_dot = 0;
        s_ddot = 0;
        d_ddot = 0;
    }
    else
    {
        double pos_x2, pos_y2, vel_x1, vel_y1, pos_x3, pos_y3, vel_x2, vel_y2, acc_x, acc_y;
        // consider current position to be last point of previous path to be kept
        pos_x = previous_path.x[subpath_size - 1];
        pos_y = previous_path.y[subpath_size - 1];
        pos_x2 = previous_path.x[subpath_size - 2];
        pos_y2 = previous_path.y[subpath_size - 2];
        angle = atan2(pos_y - pos_y2, pos_x - pos_x2);
        vector<double> frenet = getFrenet(pos_x,
                                          pos_y,
                                          angle,
                                          interpolated_waypoints.x,
                                          interpolated_waypoints.y,
                                          interpolated_waypoints.s);
        pos_s = frenet[0];
        pos_d = frenet[1];

        // determine dx, dy vector from set of interpoated waypoints
        int next_interp_waypoint_index = nextWaypoint(pos_x, pos_y,
                                                      angle,
                                                      interpolated_waypoints.x,
                                                      interpolated_waypoints.y);
        double dx = interpolated_waypoints.dx[next_interp_waypoint_index - 1];
        double dy = interpolated_waypoints.dy[next_interp_waypoint_index - 1];

        // sx,sy vector is perpendicular to dx,dy
        double sx = -dy;
        double sy = dx;

        vel_x1 = (pos_x - pos_x2) / Planner::dt;
        vel_y1 = (pos_y - pos_y2) / Planner::dt;
        s_dot = vel_x1 * sx + vel_y1 * sy;
        d_dot = vel_x1 * dx + vel_y1 * dy;

        // have to get another point to calculate s_ddot, d_ddot from xy acceleration
        pos_x3 = previous_path.x[subpath_size - 3];
        pos_y3 = previous_path.y[subpath_size - 3];
        vel_x2 = (pos_x2 - pos_x3) / Planner::dt;
        vel_y2 = (pos_y2 - pos_y3) / Planner::dt;
        acc_x = (vel_x1 - vel_x2) / Planner::dt;
        acc_y = (vel_y1 - vel_y2) / Planner::dt;
        s_ddot = acc_x * sx + acc_y * sy;
        d_ddot = acc_x * dx + acc_y * dy;
    }

    x = pos_x;
    y = pos_y;
    yaw = angle;
    s = pos_s;
    s_d = s_dot;
    s_dd = s_ddot;
    d = pos_d;
    d_d = d_dot;
    d_dd = d_ddot;
}

void PathPlanner::checkSurrounding()
{
    surrouding.reset();

    for (Detection &car : sensor_detections)
    {
        double s_diff = fabs(car.s - s);
        if (s_diff < Planner::kSaveDistance)
        {
            double d_diff = car.d - d;
            if (d_diff > 0.0F && fabs(d_diff) < 1.5 * Road::kWidth)
            {
                surrouding.car_on_right = true;
            }
            else if (d_diff < 0.0F && fabs(d_diff) < 1.5 * Road::kWidth)
            {
                surrouding.car_on_left = true;
            }
            else if (fabs(d_diff) < 0.5 * Road::kWidth)
            {
                surrouding.car_ahead = true;
            }
        }
    }
}

void PathPlanner::predictTraffic()
{
    double traj_start_time = subpath_size * Planner::dt;
    double duration = Planner::kNumSamples * Planner::kSampleDt - subpath_size * Planner::dt;
    predictions.clear();
    for (Detection &car_deteceted : sensor_detections)
    {
        double other_car_vel = sqrt(pow((double)car_deteceted.vx, 2) + pow((double)car_deteceted.vy, 2));
        int v_id = car_deteceted.id;

        std::vector<std::vector<double>> prediction{};
        for (int i = 0; i < Planner::kNumSamples; i++)
        {
            double t = traj_start_time + (i * duration / Planner::kNumSamples);
            double s_pred = car_deteceted.s + other_car_vel * t;
            vector<double> s_and_d = {s_pred, car_deteceted.d};
            prediction.push_back(s_and_d);
        }

        predictions[v_id] = prediction;
    }
}

void PathPlanner::updateBehaviourList()
{
    behaviour_list = {"KeepLane"};
    if (d > 4 && !surrouding.car_on_left)
    {
        behaviour_list.push_back("ChangeLeft");
    }
    if (d < 8 && !surrouding.car_on_right)
    {
        behaviour_list.push_back("ChangeRight");
    }
}

std::vector<std::vector<double>> PathPlanner::calculateTrajectory(std::vector<std::vector<double>> target, double duration)
{

    vector<double> target_s = target[0];
    vector<double> target_d = target[1];
    vector<double> current_s = {s, s_d, s_dd};
    vector<double> current_d = {d, d_d, d_dd};

    // determine coefficients of optimal JMT
    s_traj_coeffs = TrajectoryHelper::getTrajCoeffs(current_s, target_s, duration);
    d_traj_coeffs = TrajectoryHelper::getTrajCoeffs(current_d, target_d, duration);

    vector<double> s_traj;
    vector<double> d_traj;

    for (int i = 0; i < Planner::kNumSamples; i++)
    {
        double t = i * duration / Planner::kNumSamples;
        double s_val = 0, d_val = 0;
        for (int j = 0; j < s_traj_coeffs.size(); j++)
        {
            s_val += s_traj_coeffs[j] * pow(t, j);
            d_val += d_traj_coeffs[j] * pow(t, j);
        }
        s_traj.push_back(s_val);
        d_traj.push_back(d_val);
    }

    return {s_traj, d_traj};
}

std::vector<std::vector<double>> PathPlanner::calculateTarget(std::string behaviour,
                                                              std::map<int, std::vector<std::vector<double>>> predictions,
                                                              double duration)
{
    // Desire :
    // lateral displacement : depends on behaviour
    // lateral velocity : 0
    int target_lane, current_lane = d / Road::kWidth;
    double target_d;
    double target_d_d = 0;
    double target_d_dd = 0;
    double target_s_d = std::min((float)(s_d + kAccelerationLimit / Road::kWidth * duration), Road::kSpeedLimit);
    target_s_d = Road::kSpeedLimit;
    double target_s_dd = 0;
    double target_s = s + (s_d + target_s_d) / 2 * duration;

    vector<double> leading_vehicle_s_and_sdot;

    if (behaviour.compare("KeepLane") == 0)
    {
        target_d = (double)current_lane * Road::kWidth + 0.5 * Road::kWidth;
        target_lane = target_d / Road::kWidth;
    }
    else if (behaviour.compare("ChangeLeft") == 0)
    {
        target_d = ((double)current_lane - 1) * Road::kWidth + 0.5 * Road::kWidth;
        target_lane = target_d / Road::kWidth;
    }
    else if (behaviour.compare("ChangeRight") == 0)
    {
        target_d = ((double)current_lane + 1) * Road::kWidth + 0.5 * Road::kWidth;
        target_lane = target_d / Road::kWidth;
    }

    // replace target_s variables if there is a leading vehicle close enough
    leading_vehicle_s_and_sdot = TrajectoryHelper::leadingCarInfo(target_lane, predictions, duration, s);
    double leading_vehicle_s = leading_vehicle_s_and_sdot[0];
    if (leading_vehicle_s - target_s < Planner::kSaveDistance && leading_vehicle_s > s)
    {

        target_s_d = leading_vehicle_s_and_sdot[1];

        if (fabs(leading_vehicle_s - target_s) < 0.5 * Planner::kSaveDistance)
        {
            target_s_d -= 1;
        }

        target_s = leading_vehicle_s - Planner::kSaveDistance;
    }

    // emergency brake
    if (surrouding.car_ahead)
    {
        target_s_d = 0.0;
    }

    return {{target_s, target_s_d, target_s_dd}, {target_d, target_d_d, target_d_dd}};
}

double PathPlanner::calculateCost(std::vector<double> s_traj,
                                  std::vector<double> d_traj,
                                  std::map<int, std::vector<std::vector<double>>> predictions)
{
    double cc = CostHelper::checkCollision(s_traj, d_traj, predictions) * Cost::kCollisionCost;
    double lc = CostHelper::leadingCarCost(s_traj, d_traj, predictions) * Cost::kLeadingCarCost;
    double ec = CostHelper::efficiencyCost(s_traj) * Cost::kEfficiencyCost;
    double nm = CostHelper::noneMiddleLaneCost(d_traj) * Cost::kNoneMiddleLaneCost;

    return cc + lc + ec + nm;
}

vector<vector<double>> PathPlanner::bestTarget()
{
    vector<vector<double>> best_target;
    double best_cost = std::numeric_limits<double>::max();
    double duration = Planner::kNumSamples * Planner::kSampleDt - subpath_size * Planner::dt;
    for (std::string behaviour : behaviour_list)
    {
        vector<vector<double>> target_s_and_d = calculateTarget(behaviour,
                                                                predictions,
                                                                duration);

        vector<vector<double>> possible_traj = calculateTrajectory(target_s_and_d, duration);

        double current_cost = calculateCost(possible_traj[0], possible_traj[1], predictions);
        if (current_cost < best_cost)
        {
            best_cost = current_cost;
            best_target = target_s_and_d;
        }
    }

    return best_target;
}

void PathPlanner::generateNewPath(const vector<vector<double>> &target,
                                  const Waypoints &interpolated_waypoints,
                                  const Waypoints &previous_path,
                                  vector<double> &next_x_vals,
                                  vector<double> &next_y_vals)
{
    vector<double> coarse_s_traj, coarse_x_traj, coarse_y_traj, interpolated_s_traj,
        interpolated_x_traj, interpolated_y_traj;

    double prev_s = s - s_d * Planner::dt;

    // first two points of coarse trajectory, to ensure spline begins smoothly
    if (subpath_size >= 2)
    {
        coarse_s_traj.push_back(prev_s);
        coarse_x_traj.push_back(previous_path.x[subpath_size - 2]);
        coarse_y_traj.push_back(previous_path.y[subpath_size - 2]);
        coarse_s_traj.push_back(s);
        coarse_x_traj.push_back(previous_path.x[subpath_size - 1]);
        coarse_y_traj.push_back(previous_path.y[subpath_size - 1]);
    }
    else
    {
        double prev_s = s - 1;
        double prev_x = x - cos(yaw);
        double prev_y = y - sin(yaw);
        coarse_s_traj.push_back(prev_s);
        coarse_x_traj.push_back(prev_x);
        coarse_y_traj.push_back(prev_y);
        coarse_s_traj.push_back(s);
        coarse_x_traj.push_back(x);
        coarse_y_traj.push_back(y);
    }

    // last two points of coarse trajectory, use target_d and current s + 30,60
    double target_s1 = s + Road::kIntervalS;
    double target_d1 = target[1][0];
    vector<double> target_xy1 = getXY(target_s1, target_d1,
                                      interpolated_waypoints.s,
                                      interpolated_waypoints.x,
                                      interpolated_waypoints.y);
    double target_x1 = target_xy1[0];
    double target_y1 = target_xy1[1];
    coarse_s_traj.push_back(target_s1);
    coarse_x_traj.push_back(target_x1);
    coarse_y_traj.push_back(target_y1);
    double target_s2 = target_s1 + Road::kIntervalS;
    double target_d2 = target_d1;
    vector<double> target_xy2 = getXY(target_s2,
                                      target_d2,
                                      interpolated_waypoints.s,
                                      interpolated_waypoints.x,
                                      interpolated_waypoints.y);
    double target_x2 = target_xy2[0];
    double target_y2 = target_xy2[1];
    coarse_s_traj.push_back(target_s2);
    coarse_x_traj.push_back(target_x2);
    coarse_y_traj.push_back(target_y2);

    // next s values
    double target_s_dot = target[0][1];
    double current_s = s;
    double current_v = s_d;
    for (int i = 0; i < (Planner::kNumPathPoints - subpath_size); i++)
    {
        double v_incr, a_incr;
        if (fabs(target_s_dot - current_v) < 2 * Cost::kVelocityIncrementLimit)
        {
            v_incr = 0;
        }
        else
        {
            v_incr = (target_s_dot - current_v) / (fabs(target_s_dot - current_v)) * Cost::kVelocityIncrementLimit;
        }
        current_v += v_incr;
        current_s += current_v * Planner::dt;
        interpolated_s_traj.push_back(current_s);
    }

    interpolated_x_traj = interpolate(coarse_s_traj, coarse_x_traj, interpolated_s_traj);
    interpolated_y_traj = interpolate(coarse_s_traj, coarse_y_traj, interpolated_s_traj);

    // add previous path, if any, to next path
    for (int i = 0; i < subpath_size; i++)
    {
        next_x_vals.push_back(previous_path.x[i]);
        next_y_vals.push_back(previous_path.y[i]);
    }
    // add xy points from newly generated path
    for (int i = 0; i < interpolated_x_traj.size(); i++)
    {
        next_x_vals.push_back(interpolated_x_traj[i]);
        next_y_vals.push_back(interpolated_y_traj[i]);
    }
}

void PathPlanner::planPath(const Waypoints &map,
                           const Waypoints &previous_path,
                           std::vector<double> &next_x_vals,
                           std::vector<double> &next_y_vals)
{
    Waypoints closest_waypoints = detectClosestWaypoints(map);
    Waypoints interpolated_waypoints = interpolateWaypoints(closest_waypoints);
    updateEgoCarState(interpolated_waypoints, previous_path);
    predictTraffic();
    checkSurrounding();
    updateBehaviourList();
    vector<vector<double>> target = bestTarget();
    generateNewPath(target, interpolated_waypoints, previous_path, next_x_vals, next_y_vals);
}