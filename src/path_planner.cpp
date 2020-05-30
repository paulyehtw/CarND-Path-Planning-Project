#include "path_planner.h"
#include "helpers.h"
#include <iostream>

Waypoints PathPlanner::detectClosestWaypoints(const Car &ego_car_state,
                                              const Waypoints &map)
{
    int num_waypoints = map.x.size();
    int next_waypoint_index = NextWaypoint(ego_car_state.x,
                                           ego_car_state.y,
                                           ego_car_state.yaw,
                                           map.x,
                                           map.y);

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
        closest_waypoints.x.push_back(map.x[idx]);
        closest_waypoints.y.push_back(map.y[idx]);
        closest_waypoints.s.push_back(map.s[idx]);
        closest_waypoints.dx.push_back(map.dx[idx]);
        closest_waypoints.dy.push_back(map.dy[idx]);
    }
    return closest_waypoints;
}

Waypoints PathPlanner::interpolateWaypoints(const Waypoints &waypoints)
{
    Waypoints interpolated_waypoints{};
    float interval = PlannerParameter::kInterpolatedWaypointsInterval;
    double s_length = (waypoints.s[waypoints.s.size() - 1] - waypoints.s[0]); // use s to calcualte interpolation points
    int num_interpolation_points = s_length / interval;

    // Interpolate s
    interpolated_waypoints.s.push_back(waypoints.s[0]);
    for (int i = 1; i < num_interpolation_points; i++)
    {
        interpolated_waypoints.s.push_back(waypoints.s[i - 1] + interval);
    }
    // Interpolate  the rest
    interpolated_waypoints.x = interpolate(waypoints.s, waypoints.x, interval, num_interpolation_points);
    interpolated_waypoints.y = interpolate(waypoints.s, waypoints.y, interval, num_interpolation_points);
    interpolated_waypoints.dx = interpolate(waypoints.s, waypoints.dx, interval, num_interpolation_points);
    interpolated_waypoints.dy = interpolate(waypoints.s, waypoints.dy, interval, num_interpolation_points);

    return interpolated_waypoints;
}

int PathPlanner::updateCoefficients(Car &ego_car_state,
                                    const Waypoints &interpolated_waypoints,
                                    const Waypoints &previous_path)
{
    double pos_s, s_dot, s_ddot;
    double pos_d, d_dot, d_ddot;
    // Other values necessary for determining these based on future points in previous path
    double pos_x, pos_y, pos_x2, pos_y2, angle, vel_x1, vel_y1,
        pos_x3, pos_y3, vel_x2, vel_y2, acc_x, acc_y;

    int subpath_size = std::min((int)PlannerParameter::kNumKeptPreviousPath, (int)previous_path.x.size());
    double traj_start_time = subpath_size * PlannerParameter::dt;

    // use default values if not enough previous path points
    if (subpath_size < 4)
    {
        pos_s = ego_car_state.s;
        pos_d = ego_car_state.d;
        s_dot = ego_car_state.v;
        d_dot = 0;
        s_ddot = 0;
        d_ddot = 0;
    }
    else
    {
        // consider current position to be last point of previous path to be kept
        pos_x = previous_path.x[subpath_size - 1];
        pos_y = previous_path.y[subpath_size - 1];
        pos_x2 = previous_path.x[subpath_size - 2];
        pos_y2 = previous_path.y[subpath_size - 2];
        angle = atan2(pos_y - pos_y2, pos_x - pos_x2);
        vector<double> frenet = getFrenet(pos_x, pos_y, angle, interpolated_waypoints.x, interpolated_waypoints.y);
        pos_s = frenet[0];
        pos_d = frenet[1];

        // determine dx, dy vector from set of interpoated waypoints, with pos_x,pos_y as reference point;
        // since interpolated waypoints are ~1m apart and path points tend to be <0.5m apart, these
        // values can be reused for previous two points (and using the previous waypoint data may be
        // more accurate) to calculate vel_s (s_dot), vel_d (d_dot), acc_s (s_ddot), and acc_d (d_ddot)
        int next_interp_waypoint_index = NextWaypoint(pos_x, pos_y,
                                                      angle,
                                                      interpolated_waypoints.x,
                                                      interpolated_waypoints.y);
        double dx = interpolated_waypoints.dx[next_interp_waypoint_index - 1];
        double dy = interpolated_waypoints.dy[next_interp_waypoint_index - 1];
        // sx,sy vector is perpendicular to dx,dy
        double sx = -dy;
        double sy = dx;

        // calculate s_dot & d_dot
        vel_x1 = (pos_x - pos_x2) / PlannerParameter::dt;
        vel_y1 = (pos_y - pos_y2) / PlannerParameter::dt;
        // want projection of xy velocity vector (V) onto S (sx,sy) and D (dx,dy) vectors, and since S
        // and D are unit vectors this is simply the dot products of V with S and V with D
        s_dot = vel_x1 * sx + vel_y1 * sy;
        d_dot = vel_x1 * dx + vel_y1 * dy;

        // have to get another point to calculate s_ddot, d_ddot from xy acceleration
        pos_x3 = previous_path.x[subpath_size - 3];
        pos_y3 = previous_path.y[subpath_size - 3];
        vel_x2 = (pos_x2 - pos_x3) / PlannerParameter::dt;
        vel_y2 = (pos_y2 - pos_y3) / PlannerParameter::dt;
        acc_x = (vel_x1 - vel_x2) / PlannerParameter::dt;
        acc_y = (vel_y1 - vel_y2) / PlannerParameter::dt;
        s_ddot = acc_x * sx + acc_y * sy;
        d_ddot = acc_x * dx + acc_y * dy;

        // try, instead, differentiating trajectory coefficients
        double eval_time, pos_s2, pos_d2, s_dot2, d_dot2, s_ddot2, d_ddot2;
        vector<double> s_dot_coeffs = differentiate(s_traj_coeffs);
        vector<double> d_dot_coeffs = differentiate(d_traj_coeffs);
        vector<double> s_ddot_coeffs = differentiate(s_dot_coeffs);
        vector<double> d_ddot_coeffs = differentiate(d_dot_coeffs);
        eval_time = (PlannerParameter::kNumPathPoints - subpath_size) * PlannerParameter::dt;
        pos_s2 = function(s_traj_coeffs, eval_time);
        pos_d2 = function(d_traj_coeffs, eval_time);
        s_dot2 = function(s_dot_coeffs, eval_time);
        d_dot2 = function(d_dot_coeffs, eval_time);
        s_ddot2 = function(s_ddot_coeffs, eval_time);
        d_ddot2 = function(d_ddot_coeffs, eval_time);
    }

    s = pos_s;     // s position
    s_d = s_dot;   // s dot - velocity in s
    s_dd = s_ddot; // s dot-dot - acceleration in s
    d = pos_d;     // d position
    d_d = d_dot;   // d dot - velocity in d
    d_dd = d_ddot; // d dot-dot - acceleration in d

    return subpath_size;
}

void PathPlanner::detectTraffic(const std::vector<Car> &traffic, const Car &ego_car_state)
{
    bool car_to_left = false, car_to_right = false, car_just_ahead = false;
    for (Car car : traffic)
    {
        double s_diff = fabs(car.s - ego_car_state.s);
        if (s_diff < PlannerParameter::kMinLeadingDistance)
        {
            double d_diff = car.d - ego_car_state.d;
            if (d_diff > 2 && d_diff < 6)
            {
                car_to_right = true;
            }
            else if (d_diff < -2 && d_diff > -6)
            {
                car_to_left = true;
            }
            else if (d_diff > -2 && d_diff < 2)
            {
                car_just_ahead = true;
            }
        }
    }

    if (car_to_right)
        std::cout << "CAR ON THE RIGHT!!!" << std::endl;
    if (car_to_left)
        std::cout << "CAR ON THE LEFT!!!" << std::endl;
    if (car_just_ahead)
        std::cout << "CAR JUST AHEAD!!!" << std::endl;
}

void PathPlanner::predictTraffic(const std::vector<Car> &traffic, const int &subpath_size)
{
    double traj_start_time = subpath_size * PlannerParameter::dt;
    double duration = PlannerParameter::kNumSamples * PlannerParameter::kSampleDt - subpath_size * PlannerParameter::dt;
    for (int i = 0; i < traffic.size(); ++i)
    {
        std::vector<std::pair<double, double>> prediction{};
        for (int j = 0; j < PlannerParameter::kNumSamples; j++)
        {
            double t = traj_start_time + (j * duration / PlannerParameter::kNumSamples);
            double s_pred = s + s_d * t;
            prediction.push_back(std::make_pair(s_pred, d));
        }
        traffic_predictions[i] = prediction;
    }
}