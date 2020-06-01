#include "path_planner.h"
#include "helpers.h"
#include <iostream>
#include <limits>

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
            current_s -= TRACK_LENGTH;
        }
        if (i > 0 && current_s < base_s)
        {
            current_s += TRACK_LENGTH;
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
    float interval = PlannerParameter::kInterpolatedWaypointsInterval;
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

int PathPlanner::updateCoefficients(Car &ego_car_state,
                                    const Waypoints &interpolated_waypoints,
                                    const Waypoints &previous_path)
{
    double pos_s, s_dot, s_ddot;
    double pos_d, d_dot, d_ddot;
    // Other values necessary for determining these based on future points in previous path
    double pos_x, pos_y, pos_x2, pos_y2, angle, vel_x1, vel_y1,
        pos_x3, pos_y3, vel_x2, vel_y2, acc_x, acc_y;

    subpath_size = std::min((int)PlannerParameter::kNumKeptPreviousPath, (int)previous_path.x.size());
    double traj_start_time = subpath_size * PlannerParameter::dt;

    // use default values if not enough previous path points
    if (subpath_size < 4)
    {
        pos_x = ego_car_state.x;
        pos_y = ego_car_state.y;
        angle = deg2rad(ego_car_state.yaw);
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
        vector<double> frenet = getFrenet(pos_x, pos_y, angle, interpolated_waypoints.x, interpolated_waypoints.y, interpolated_waypoints.s);
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
        pos_s2 = value(s_traj_coeffs, eval_time);
        pos_d2 = value(d_traj_coeffs, eval_time);
        s_dot2 = value(s_dot_coeffs, eval_time);
        d_dot2 = value(d_dot_coeffs, eval_time);
        s_ddot2 = value(s_ddot_coeffs, eval_time);
        d_ddot2 = value(d_ddot_coeffs, eval_time);
    }

    x = pos_x;
    y = pos_y;
    yaw = angle;
    s = pos_s;     // s position
    s_d = s_dot;   // s dot - velocity in s
    s_dd = s_ddot; // s dot-dot - acceleration in s
    d = pos_d;     // d position
    d_d = d_dot;   // d dot - velocity in d
    d_dd = d_ddot; // d dot-dot - acceleration in d
}

void PathPlanner::detectTraffic(const std::vector<CarDetected> &sensor_detections, const Car &ego_car_state)
{
    traffic_states.reset();

    for (CarDetected car : sensor_detections)
    {
        double s_diff = fabs(car.s - ego_car_state.s);
        if (s_diff < PlannerParameter::kMinLeadingDistance)
        {
            double d_diff = car.d - ego_car_state.d;
            if (d_diff > 2 && d_diff < 6)
            {
                traffic_states.car_on_right = true;
            }
            else if (d_diff < -2 && d_diff > -6)
            {
                traffic_states.car_on_left = true;
            }
            else if (d_diff > -2 && d_diff < 2)
            {
                traffic_states.car_ahead = true;
            }
        }
    }
}

void PathPlanner::predictTraffic()
{
    double traj_start_time = subpath_size * PATH_DT;
    double duration = N_SAMPLES * DT - subpath_size * PATH_DT;
    vector<Car> other_cars;
    // std::map<int, vector<vector<double>>> predictions;
    traffic_predictions.clear();
    double other_car_vel_test = 0;

    for (CarDetected &car_deteceted : sensor_detections)
    {
        double other_car_vel = sqrt(pow((double)car_deteceted.vx, 2) + pow((double)car_deteceted.vy, 2));
        other_car_vel_test += other_car_vel;
        Car car = Car(0.0F, 0.0F, car_deteceted.s, car_deteceted.d, other_car_vel, 0.0F);
        other_cars.push_back(car);
        int v_id = car_deteceted.id;

        std::vector<std::vector<double>> prediction{};
        for (int i = 0; i < PlannerParameter::kNumSamples; i++)
        {
            double t = traj_start_time + (i * duration / PlannerParameter::kNumSamples);
            double s_pred = car.s + car.v * t;
            vector<double> s_and_d = {s_pred, car.d};
            prediction.push_back(s_and_d);
        }

        traffic_predictions[v_id] = prediction;
    }
    // return predictions;
}

void PathPlanner::updateStates()
{
    available_states = {"KL"};
    if (d > 4 && !traffic_states.car_on_left)
    {
        available_states.push_back("LCL");
    }
    if (d < 8 && !traffic_states.car_on_right)
    {
        available_states.push_back("LCR");
    }
}

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

std::vector<std::vector<double>> PathPlanner::calculateTrajectory(std::vector<std::vector<double>> target, double duration)
{
    // takes a target {{s, s_dot, s_ddot}, {d, d_dot, d_ddot}} and returns a Jerk-Minimized Trajectory
    // (JMT) connecting current state (s and d) to target state in a list of s points and a list of d points
    // ex. {{s1, s2, ... , sn}, {d1, d2, ... , dn}}
    vector<double> target_s = target[0];
    vector<double> target_d = target[1];
    vector<double> current_s = {this->s, this->s_d, this->s_dd};
    vector<double> current_d = {this->d, this->d_d, this->d_dd};

    // determine coefficients of optimal JMT
    this->s_traj_coeffs = getTrajCoeffs(current_s, target_s, duration);
    this->d_traj_coeffs = getTrajCoeffs(current_d, target_d, duration);

    // // DEBUG
    // cout << "s coeffs: ";
    // for (auto s : this->s_traj_coeffs) cout << s << ",";
    // cout << endl;
    // cout << "d coeffs: ";
    // for (auto d : this->d_traj_coeffs) cout << d << ",";
    // cout << endl << endl;

    vector<double> s_traj;
    vector<double> d_traj;

    // populate s and t trajectories at each time step
    for (int i = 0; i < PlannerParameter::kNumSamples; i++)
    {
        double t = i * duration / PlannerParameter::kNumSamples;
        double s_val = 0, d_val = 0;
        for (int j = 0; j < s_traj_coeffs.size(); j++)
        {
            s_val += this->s_traj_coeffs[j] * pow(t, j);
            d_val += this->d_traj_coeffs[j] * pow(t, j);
        }
        s_traj.push_back(s_val);
        d_traj.push_back(d_val);
    }

    return {s_traj, d_traj};
}

std::vector<double> PathPlanner::get_leading_vehicle_data_for_lane(int target_lane,
                                                                   std::map<int, std::vector<std::vector<double>>> predictions,
                                                                   double duration)
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
            double dt = duration / PlannerParameter::kNumSamples;
            double predicted_s_dot = (predicted_end_s - next_to_last_s) / dt;
            if (predicted_end_s < nearest_leading_vehicle_distance && start_s > this->s)
            {
                nearest_leading_vehicle_distance = predicted_end_s;
                nearest_leading_vehicle_speed = predicted_s_dot;
            }
        }
    }
    return {nearest_leading_vehicle_distance, nearest_leading_vehicle_speed};
}

std::vector<std::vector<double>> PathPlanner::calculateTarget(std::string state,
                                                              std::map<int, std::vector<std::vector<double>>> predictions,
                                                              double duration,
                                                              bool car_just_ahead)
{
    // Returns two lists s_target and d_target in a single vector - s_target includes
    // [s, s_dot, and s_ddot] and d_target includes the same
    // If no leading car found target lane, ego car will make up PERCENT_V_DIFF_TO_MAKE_UP of the difference
    // between current velocity and target velocity. If leading car is found set target s to FOLLOW_DISTANCE
    // and target s_dot to leading car's s_dot based on predictions
    int target_lane, current_lane = this->d / 4;
    double target_d;
    // **** TARGETS ****
    // lateral displacement : depends on state
    // lateral velocity : 0
    double target_d_d = 0;
    // lateral acceleration : 0
    double target_d_dd = 0;
    // longitudinal velocity : current velocity + max allowed accel * duration
    double target_s_d = std::min(this->s_d + MAX_INSTANTANEOUS_ACCEL / 4 * duration, SPEED_LIMIT);
    target_s_d = SPEED_LIMIT;
    // longitudinal acceleration : zero ?
    double target_s_dd = 0;
    // longitudinal acceleration : difference between current/target velocity over trajectory duration?
    //double target_s_dd = (target_s_d - this->s_d) / (N_SAMPLES * DT);
    // longitudinal displacement : current displacement plus difference in current/target velocity times
    // trajectory duration
    double target_s = this->s + (this->s_d + target_s_d) / 2 * duration;

    vector<double> leading_vehicle_s_and_sdot;

    if (state.compare("KL") == 0)
    {
        target_d = (double)current_lane * 4 + 2;
        target_lane = target_d / 4;
    }
    else if (state.compare("LCL") == 0)
    {
        target_d = ((double)current_lane - 1) * 4 + 2;
        target_lane = target_d / 4;
    }
    else if (state.compare("LCR") == 0)
    {
        target_d = ((double)current_lane + 1) * 4 + 2;
        target_lane = target_d / 4;
    }

    // replace target_s variables if there is a leading vehicle close enough
    leading_vehicle_s_and_sdot = get_leading_vehicle_data_for_lane(target_lane, predictions, duration);
    double leading_vehicle_s = leading_vehicle_s_and_sdot[0];
    if (leading_vehicle_s - target_s < PlannerParameter::kMinLeadingDistance && leading_vehicle_s > this->s)
    {

        target_s_d = leading_vehicle_s_and_sdot[1];

        if (fabs(leading_vehicle_s - target_s) < 0.5 * PlannerParameter::kMinLeadingDistance)
        {
            target_s_d -= 1;
        }

        target_s = leading_vehicle_s - PlannerParameter::kMinLeadingDistance;
    }

    // emergency brake
    if (car_just_ahead)
    {
        target_s_d = 0.0;
    }

    return {{target_s, target_s_d, target_s_dd}, {target_d, target_d_d, target_d_dd}};
}

double PathPlanner::calculateCost(std::vector<double> s_traj,
                                  std::vector<double> d_traj,
                                  std::map<int, std::vector<std::vector<double>>> predictions)
{
    double total_cost = 0;
    double col = CostCalculatorHelper::collision_cost(s_traj, d_traj, predictions) * COLLISION_COST_WEIGHT;
    double buf = CostCalculatorHelper::buffer_cost(s_traj, d_traj, predictions) * BUFFER_COST_WEIGHT;
    double ilb = CostCalculatorHelper::in_lane_buffer_cost(s_traj, d_traj, predictions) * IN_LANE_BUFFER_COST_WEIGHT;
    double eff = CostCalculatorHelper::efficiency_cost(s_traj) * EFFICIENCY_COST_WEIGHT;
    double nml = CostCalculatorHelper::not_middle_lane_cost(d_traj) * NOT_MIDDLE_LANE_COST_WEIGHT;

    total_cost += col + buf + ilb + eff + nml; // + esl + mas + aas + mad + aad + mjs + ajs + mjd + ajd;

    return total_cost;
}

vector<vector<double>> PathPlanner::generateTarget()
{
    vector<vector<double>> best_target;
    double best_cost = 999999;
    std::string best_traj_state = "";
    // double duration = PlannerParameter::kNumSamples * PlannerParameter::kSampleDt - subpath_size * PlannerParameter::dt;
    double duration = N_SAMPLES * DT - subpath_size * PATH_DT;
    for (std::string state : available_states)
    {
        vector<vector<double>> target_s_and_d = calculateTarget(state,
                                                                traffic_predictions,
                                                                duration,
                                                                traffic_states.car_ahead);

        vector<vector<double>> possible_traj = calculateTrajectory(target_s_and_d, duration);

        double current_cost = calculateCost(possible_traj[0], possible_traj[1], traffic_predictions);
        if (current_cost < best_cost)
        {
            best_cost = current_cost;
            best_traj_state = state;
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
    // begin by pushing the last and next-to-last point from the previous path for setting the
    // spline the last point should be the first point in the returned trajectory, but because of
    // imprecision, also add that point manually

    vector<double> coarse_s_traj, coarse_x_traj, coarse_y_traj, interpolated_s_traj,
        interpolated_x_traj, interpolated_y_traj;

    double prev_s = s - s_d * PlannerParameter::dt;

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
    double target_s1 = s + 30;
    double target_d1 = target[1][0];
    vector<double> target_xy1 = getXY(target_s1, target_d1, interpolated_waypoints.s, interpolated_waypoints.x, interpolated_waypoints.y);
    double target_x1 = target_xy1[0];
    double target_y1 = target_xy1[1];
    coarse_s_traj.push_back(target_s1);
    coarse_x_traj.push_back(target_x1);
    coarse_y_traj.push_back(target_y1);
    double target_s2 = target_s1 + 30;
    double target_d2 = target_d1;
    vector<double> target_xy2 = getXY(target_s2, target_d2, interpolated_waypoints.s, interpolated_waypoints.x, interpolated_waypoints.y);
    double target_x2 = target_xy2[0];
    double target_y2 = target_xy2[1];
    coarse_s_traj.push_back(target_s2);
    coarse_x_traj.push_back(target_x2);
    coarse_y_traj.push_back(target_y2);

    // next s values
    double target_s_dot = target[0][1];
    double current_s = s;
    double current_v = s_d;
    double current_a = s_dd;
    for (int i = 0; i < (PlannerParameter::kNumPathPoints - subpath_size); i++)
    {
        double v_incr, a_incr;
        if (fabs(target_s_dot - current_v) < 2 * VELOCITY_INCREMENT_LIMIT)
        {
            v_incr = 0;
        }
        else
        {
            // arrived at VELOCITY_INCREMENT_LIMIT value empirically
            v_incr = (target_s_dot - current_v) / (fabs(target_s_dot - current_v)) * VELOCITY_INCREMENT_LIMIT;
        }
        current_v += v_incr;
        current_s += current_v * PlannerParameter::dt;
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
        //if (subpath_size == 0 && i == 0) continue; // maybe skip start position as a path point?
        next_x_vals.push_back(interpolated_x_traj[i]);
        next_y_vals.push_back(interpolated_y_traj[i]);
    }
}