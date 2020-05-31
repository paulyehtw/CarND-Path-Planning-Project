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
    traffic_states.reset();

    for (Car car : traffic)
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

void PathPlanner::predictTraffic(const std::vector<Car> &traffic, const int &subpath_size)
{
    double traj_start_time = subpath_size * PlannerParameter::dt;
    double duration = PlannerParameter::kNumSamples * PlannerParameter::kSampleDt - subpath_size * PlannerParameter::dt;
    for (int i = 0; i < traffic.size(); ++i)
    {
        std::vector<std::vector<double>> prediction{};
        for (int j = 0; j < PlannerParameter::kNumSamples; j++)
        {
            double t = traj_start_time + (j * duration / PlannerParameter::kNumSamples);
            double s_pred = s + s_d * t;
            vector<double> s_and_d = {s_pred, d};
            prediction.push_back(s_and_d);
        }
        traffic_predictions[i] = prediction;
    }
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

std::vector<double> get_traj_coeffs(std::vector<double> start, std::vector<double> end, double T)
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
    this->s_traj_coeffs = get_traj_coeffs(current_s, target_s, duration);
    this->d_traj_coeffs = get_traj_coeffs(current_d, target_d, duration);

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
    double col = CostCalculatorHelper::collisionCost(s_traj, d_traj, predictions) * COLLISION_COST_WEIGHT;
    double buf = CostCalculatorHelper::bufferCost(s_traj, d_traj, predictions) * BUFFER_COST_WEIGHT;
    double ilb = CostCalculatorHelper::inLaneBufferCost(s_traj, d_traj, predictions) * IN_LANE_BUFFER_COST_WEIGHT;
    double eff = CostCalculatorHelper::efficiencyCost(s_traj) * EFFICIENCY_COST_WEIGHT;
    double nml = CostCalculatorHelper::notMiddleLaneCost(d_traj) * NOT_MIDDLE_LANE_COST_WEIGHT;

    total_cost += col + buf + ilb + eff + nml; // + esl + mas + aas + mad + aad + mjs + ajs + mjd + ajd;

    return total_cost;
}

void PathPlanner::generateTarget(const std::vector<Car> &traffic, const Car &ego_car_state, const int &subpath_size)
{
    vector<vector<double>> best_target;
    double best_cost = std::numeric_limits<double>::max();
    std::string best_traj_state = "";
    double duration = PlannerParameter::kNumSamples * PlannerParameter::kSampleDt - subpath_size * PlannerParameter::dt;
    detectTraffic(traffic, ego_car_state);
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
}