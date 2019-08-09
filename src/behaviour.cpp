#include <iostream>
#include <vector>
#include <math.h>
#include "behaviour.h"
#include "vehicle.h"
#include "trajectory.h"
#include "state_machine.h"
#include "path_generator.h"
#include "path_validator.h"
#include "cost_functions.h"
#include "helpers.h"
#include "collision_detector.h"

using namespace std;

Behaviour::Behaviour()
{this->current_timestep = 0;
 this->lock_timestep = 0;
 trajectory = Trajectory();}

Trajectory Behaviour::nextTrajectory(const Vehicle &ego, const vector<Vehicle> &vehicles,
                                     vector<double> &previous_path_x, vector<double> &previous_path_y)
{  Trajectory chosen_trajectory;
    State chosen_state;
    const double original_cost = 10000000000;
    double lowest_cost = original_cost;
    bool state_chosen = false;
    PathValidator path_validator = PathValidator();
    double time_interval = 1.7;
    int from_point = this->current_timestep == 0 ? 0 : 10;

    if (this->current_timestep == 0)
    {   this->trajectory.add(ego.x, ego.y, ego.s, 0.0, 0.0, 0.0, ego.d, 0.0, 0.0, 0.0, ego.theta);
        // This is the initialisation step
        int start_lane = calculateLane(this->trajectory.ds[0], DEFAULT_LANE_SPACING, DEFAULT_LANE_INSIDE_OFFSET);
        int finish_lane = calculateLane(this->trajectory.ds[this->trajectory.size() - 1], DEFAULT_LANE_SPACING, DEFAULT_LANE_INSIDE_OFFSET);
        State initial_state = State(LongitudinalState::ACCELERATE, LateralState::STAY_IN_LANE, start_lane, finish_lane);
        this->state_machine = StateMachine(initial_state);}

    int prev_path_size = previous_path_x.size();
    int points_consumed = this->trajectory.size() - prev_path_size;
    if (prev_path_size > 0)
    {this->trajectory.removeFirstN(points_consumed);}
    this->current_timestep += points_consumed;
    cout << "----------------------------- TIMESTEP " << this->current_timestep << " -----------------------------" << endl;

    vector<State> next_states = this->update(ego, vehicles, this->trajectory);
    PathGenerator path_gen = PathGenerator(ego, this->trajectory);

    int h_space = 15;
    cout << left << setw(h_space) << setfill(' ') << "State/Cost";
    cout << left << setw(h_space) << setfill(' ') << "| LANE_CENTER";
    cout << left << setw(h_space) << setfill(' ') << "| SPEED";
    cout << left << setw(h_space) << setfill(' ') << "| AVG SPEED DS";
    cout << left << setw(h_space) << setfill(' ') << "| DIST SL ";
    cout << left << setw(h_space) << setfill(' ') << "| CHANGE LANE";
    cout << left << setw(h_space) << setfill(' ') << "| DIST FL";
    cout << left << setw(h_space) << setfill(' ') << "| DIST ADJ";
    cout << left << setw(h_space) << setfill(' ') << "| DIST GOAL";
    cout << left << setw(h_space) << setfill(' ') << "| SPEED DIFF";
    cout << left << setw(h_space) << setfill(' ') << "| COLLISION";
    cout << left << setw(h_space) << setfill(' ') << "| Total";
    cout << endl;
    for (const State &state : next_states)
    {auto paths = path_gen.generatePaths(state, ego, this->trajectory, from_point, 1, time_interval);
        for (auto &path : paths)
        {PathValidationStatus path_status = path_validator.validate(ego, vehicles, state, path, 0);           
            
            if (path_status != PathValidationStatus::VALID)
            {cout << "*** IGNORING STATE - PATH VALIDATION STATUS =  for state (" << state.s_state << ", " << state.d_state << ") = "
                     << path_status << endl;
                continue;}

            CostFunction lane_center_cost_fn = centerOfLaneDistCostFunction;
            double lane_center_cost = centerOfLaneDistCostFunction(ego, vehicles, path, state, 5.0);

            CostFunction cost_speed_fn = speedCostFunction;
            double cost_speed = cost_speed_fn(ego, vehicles, path, state, 1.0);
            // double cost_speed = 0.0;

            CostFunction avg_speed_lane_diff_fn = averageLaneSpeedDiffCostFunction;
            double avg_speed_lane_diff_cost = avg_speed_lane_diff_fn(ego, vehicles, path, state, 50.0);

            CostFunction dist_cars_cost_fn = distanceToClosestCarAheadCostFunction;
            double cost_dist_cars = dist_cars_cost_fn(ego, vehicles, path, state, 100.0);

            CostFunction change_lane_cost_fn = laneChangeCostFunction;
            double change_lane_cost = change_lane_cost_fn(ego, vehicles, path, state, 5.0);
            // double change_lane_cost = 0.0;

            CostFunction future_dist_to_goal_cost_fn = futureDistanceToGoalCostFunction;
            double future_dist_to_goal_cost = future_dist_to_goal_cost_fn(ego, vehicles, path, state, 1.0);

            CostFunction speed_diff_to_car_ahead_fn = speedDifferenceWithClosestCarAheadCostFunction;
            double speed_diff_to_car_ahead_cost = speed_diff_to_car_ahead_fn(ego, vehicles, path, state, 100.0);

            CostFunction collision_time_cost_fn = collisionTimeCostFunction;
            double collision_time_cost = collision_time_cost_fn(ego, vehicles, path, state, 10000.0);

            CostFunction dist_car_future_lane_cost_fn = distanceToClosestCarAheadFutureLaneCostFunction;
            double dist_car_future_lane_cost = dist_car_future_lane_cost_fn(ego, vehicles, path, state, 1000.0);
            // double dist_car_future_lane_cost = 0.0;

            CostFunction lon_dist_adjacent_car_cost_fn = longitudinalDistanceToClosestAdjacentCarFunction;
            // double lon_dist_adjacent_car_cost = lon_dist_adjacent_car_cost_fn(ego, vehicles, path, state, 1000.0);
            double lon_dist_adjacent_car_cost = 0.0;

            double final_cost = lane_center_cost 
                                + cost_speed 
                                + avg_speed_lane_diff_cost 
                                + cost_dist_cars + change_lane_cost 
                                + future_dist_to_goal_cost 
                                + speed_diff_to_car_ahead_cost 
                                + collision_time_cost + dist_car_future_lane_cost 
                                + lon_dist_adjacent_car_cost;

            cout << left << "(" << state.s_state << "," << state.d_state << ")"
                 << ":" << state.current_lane << "->" << state.future_lane << endl;
            cout << left << setw(14) << setfill(' ') << "   Ego Lane: " << ego.lane;
            cout << left << "| " << setw(13) << setfill(' ') << lane_center_cost;
            cout << left << "| " << setw(13) << setfill(' ') << cost_speed;
            cout << left << "| " << setw(13) << setfill(' ') << avg_speed_lane_diff_cost;
            cout << left << "| " << setw(13) << setfill(' ') << cost_dist_cars;
            cout << left << "| " << setw(13) << setfill(' ') << change_lane_cost;
            cout << left << "| " << setw(13) << setfill(' ') << dist_car_future_lane_cost;
            cout << left << "| " << setw(13) << setfill(' ') << lon_dist_adjacent_car_cost;
            cout << left << "| " << setw(13) << setfill(' ') << future_dist_to_goal_cost;
            cout << left << "| " << setw(13) << setfill(' ') << speed_diff_to_car_ahead_cost;
            cout << left << "| " << setw(13) << setfill(' ') << collision_time_cost;
            cout << left << "| " << setw(13) << setfill(' ') << final_cost;
            cout << endl;

            if (final_cost < lowest_cost)
            {lowest_cost = final_cost;
                chosen_trajectory = path;
                chosen_state = state;}}}
    cout << "*  - Chosen state: (" << chosen_state.s_state
         << "," << chosen_state.d_state << ")"
         << " lane transition: " << chosen_state.current_lane << " -> " << chosen_state.future_lane
         << endl;
    
    if (lowest_cost < original_cost)
    {this->trajectory = chosen_trajectory;}
    else
    {cout << "!!!!!! No lowest cost found - using current trajectory for now !!!!!!" << endl;
        chosen_trajectory = this->trajectory;}
    this->updateState(chosen_state);
    return this->trajectory;}

vector<State> Behaviour::update(const Vehicle &ego, const vector<Vehicle> others,
                                Trajectory &current_trajectory)
{ State current_state = this->state_machine.getCurrentState();
    if (this->current_timestep < this->lock_timestep)
    { cout << "*** UPDATES FROZEN RETURNING CURRENT STATE: "
             << this->current_timestep << " < " << this->lock_timestep
             << endl; return {current_state};}
    auto next_states = this->state_machine.nextPossibleStates();

    vector<State> reachable_next_states;
    for (const State &next_state : next_states)
    {if (!isLaneValid(next_state.current_lane) || !isLaneValid(next_state.future_lane))
        {continue;}
        reachable_next_states.push_back(next_state);}
    return reachable_next_states;}

void Behaviour::updateState(State new_state)
{    const State &current_state = this->state_machine.getCurrentState();
        if ((current_state.d_state == LateralState::PREPARE_CHANGE_LANE_LEFT &&
         new_state.d_state == LateralState::CHANGE_LANE_LEFT) ||
        (current_state.d_state == LateralState::PREPARE_CHANGE_LANE_RIGHT &&
         new_state.d_state == LateralState::CHANGE_LANE_RIGHT))
    {        this->lock_timestep = this->current_timestep + 25;
        cout << "*** FREEZING state updates for " << this->lock_timestep << "timesteps";}
    this->state_machine.updateState(new_state, this->lock_timestep);}

Behaviour::~Behaviour() {}
