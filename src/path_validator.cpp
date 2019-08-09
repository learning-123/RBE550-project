//This function checks whether the path is within lanes, speed is within limits, and trajectory is of the
//correct size.
#include <iostream>
#include "path_validator.h"
#include "vehicle.h"
#include "state_machine.h"
#include "collision_detector.h"
#include "trajectory.h"
#include "helpers.h"

using namespace std;

PathValidator::PathValidator(){}

PathValidator::~PathValidator(){}

PathValidationStatus PathValidator::validate(const Vehicle &ego,
                                             vector<Vehicle> others,
                                             const State &state,
                                             const Trajectory &trajectory,
                                             int from_point) const
// Reject change lane trajectories when the speed is below 30 KM/H
//Keeps the car within lane by checking lateral states.

 if (state.d_state != LateralState::STAY_IN_LANE)
 {if (trajectory.averageSpeed(trajectory.size()) < MIN_SPEED_FOR_LANE_CHANGE_METERS_PER_SECOND)

//checks if the lane is valid for both current and future lane if not gives error outside lane.

if (!isLaneValid(state.current_lane) || !isLaneValid(state.future_lane))
    {cout << "** PATH current or future lane outside bounds: "
     << state.current_lane << " ---> " << state.current_lane
     << endl;
return PathValidationStatus::OUTSIDE_OF_LANE;}

    double total_s_acc = 0.0;
    double total_d_acc = 0.0;
    double total_s_vel = 0.0;
    double total_d_vel = 0.0;
    double total_s_jerk = 0.0;
    double total_d_jerk = 0.0;
    double prev_s_vel = 0.0;
    double prev_d_vel = 0.0;

//Here a trajectory is formed between waypoints to follow in the frenet coordinate
//The velocity and accelaration is also calculated in Frenet coordinate.
    for (int i = from_point + 1; i < trajectory.xs.size(); ++i)
    {
        double last_s = trajectory.ss[i - 1];
        double last_d = trajectory.ds[i - 1];
        double cur_s = trajectory.ss[i];
        double cur_d = trajectory.ds[i];
        double s_vel = cur_s - last_s;
        double d_vel = cur_d - last_d;
        total_s_vel += s_vel;
        total_d_vel += d_vel;
        double s_acc = i < from_point + 1 ? 0.0 : s_vel - prev_s_vel;
        double d_acc = i < from_point + 1 ? 0.0 : d_vel - prev_d_vel;
        total_s_acc += abs(s_acc);
        total_d_acc += abs(d_acc);
        prev_s_vel = s_vel;
        prev_d_vel = d_vel;      }
// Here the size of segment is calculated
    double total_acc = 0;
    double prev_velocity = 0.0;
    double total_velocity = 0.0;
    int segment_size = trajectory.size() - from_point;    
    // We are interested in the path for the last second
    for (int i = from_point + 1; i < 50; ++i)
    {   double last_x = trajectory.xs[i - 1];
        double last_y = trajectory.ys[i - 1];
        double cur_x = trajectory.xs[i];
        double cur_y = trajectory.ys[i];
        double vel = distance(last_x, last_y, cur_x, cur_y);
     // Here the velocity of vehicle is checked if it above legal speed limit. 
        if (vel / CONTROLLER_UPDATE_RATE_SECONDS > MAX_LEGAL_SPEED_LIMIT_METERS_PER_SECOND)
        {cout << "**** MAXIMUM VELOCITY ABOVE THRESHOLD = " << vel / CONTROLLER_UPDATE_RATE_SECONDS << endl;
            return PathValidationStatus::VELOCITY_ABOVE_THRESHOLD;}
        total_velocity += vel;
        double acc = i == from_point ? 0.0 : vel - prev_velocity;
        total_acc += acc;
        prev_velocity = vel;    }
    double avg_velocity = total_velocity / (segment_size * CONTROLLER_UPDATE_RATE_SECONDS);    
    return PathValidationStatus::VALID;}
