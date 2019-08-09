#include <iostream>
#include <vector>
#include <functional>
#include <math.h>
#include "cost_functions.h"
#include "vehicle.h"
#include "trajectory.h"
#include "state_machine.h"
#include "collision_detector.h"
#include "helpers.h"
//These cost funcations are basically the main motion planning algorithms 
//which gives costs of certain manoveours till we get to our destination.

////////////////////////////////////////////////////////////////
///////////MOTION PLANNING USING COST FUNCTIONS////////////////
///////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////
double speedCostFunction(const Vehicle &ego, const vector<Vehicle> &others, 
const Trajectory &trajectory,const State &state, const double &weight)
{double avg_speed = trajectory.averageSpeed(CONTROLLER_UPDATE_RATE_SECONDS);
double epislon = 0.001;
if (avg_speed > MAX_SPEED_METERS_PER_SECOND + epislon)
    {return weight;}
    double diff = (MAX_SPEED_METERS_PER_SECOND - avg_speed) 
/ MAX_SPEED_METERS_PER_SECOND;
    return weight * (1 - exp(-abs(diff)));}

/////////////////////////////////////////////////////////////////////
double centerOfLaneDistCostFunction(const Vehicle &ego, const 
vector<Vehicle> &others, const Trajectory &trajectory,const State &state, const double &weight)
{double final_d = trajectory.ds[trajectory.ds.size() - 1];
int lane = calculateLane(final_d, DEFAULT_LANE_SPACING, DEFAULT_LANE_INSIDE_OFFSET);
int lane_center = getLaneCenterFrenet(lane);double diff = 
lane_center - final_d;return weight * (1 - exp(-abs(diff)));}

///////////////////////////////////////////////////////////////////////

double laneChangeCostFunction(const Vehicle &ego, const vector<Vehicle> 
&others, const Trajectory &trajectory,const State &state, const double &weight)
{if (state.current_lane == state.future_lane){// No penalty if staying on the same lane
     return 0.0;}
    // Weight penalty if switching lane
    return weight;}
////////////////////////////////////////////////////////////////////////
double distanceToClosestCarAheadCostFunction(const Vehicle &ego,
 const vector<Vehicle> &others, const Trajectory &trajectory,
  const State &state, const double &weight)
{    // Find closest car ahead and get distance
    if (!ego.isInLane)
    {return weight;}
    // Find all vehicles ahead on the current lane
    vector<Vehicle> ahead = ego.ahead(others, state.future_lane);
    if (ahead.size() == 0)
    { return 0.0;}
    double min_distance = VEHICLE_DISTANCE_THRESHOLD_METERS;
    for (const Vehicle &v : ahead)
    {   double dist = distance(ego.x, ego.y, v.x, v.y);
        if (dist < min_distance)
        { min_distance = dist;}}

    double diff = (VEHICLE_DISTANCE_THRESHOLD_METERS - min_distance) 
/ VEHICLE_DISTANCE_THRESHOLD_METERS;
    return weight * (1 - exp(-abs(diff)));}
////////////////////////////////////////////////////////////////////////////
double longitudinalDistanceToClosestAdjacentCarFunction(const Vehicle &ego, 
const vector<Vehicle> &others, const Trajectory &trajectory,const State &state, 
const double &weight)
{if (state.current_lane == state.future_lane && state.current_lane == ego.lane)
    {return 0.0;}
    double min_distance = VEHICLE_DISTANCE_THRESHOLD_METERS;
    for (const Vehicle &v : others)
    {if (v.isInLane && v.lane == state.future_lane)
        {// Other car is on different lane
            double dist = distance(ego.x, ego.y, v.x, v.y);
            if (dist < min_distance){min_distance = dist;}}}
    double diff = (VEHICLE_DISTANCE_THRESHOLD_METERS - min_distance);
    return weight * (1 - exp(-abs(diff)));}
//////////////////////////////////////////////////////////////////////////////
double distanceToClosestCarAheadFutureLaneCostFunction(const Vehicle &ego, 
const vector<Vehicle> &others, const Trajectory &trajectory,
const State &state, const double &weight)
{ // Since we are not planning to switch lanes there is no cost in this case
    if (state.d_state ==  LateralState::STAY_IN_LANE)
    {return 0.0;}
    // Find closest car ahead and get distance
    if (!ego.isInLane)
    {return weight;}
    vector<Vehicle> ahead = ego.ahead(others, state.future_lane);
    double min_distance = LANE_CHANGE_VEHICLE_AHEAD_DISTANCE_THRESHOLD_METERS;
    for (const Vehicle &v : ahead)
    {   double dist = distance(ego.x, ego.y, v.x, v.y);
        if (dist < min_distance)
        {min_distance = dist;}}

    double diff_ahead = (LANE_CHANGE_VEHICLE_AHEAD_DISTANCE_THRESHOLD_METERS-min_distance)/ LANE_CHANGE_VEHICLE_AHEAD_DISTANCE_THRESHOLD_METERS;

    vector<Vehicle> behind = ego.behind(others, state.future_lane);    
    min_distance = LANE_CHANGE_VEHICLE_BEHIND_DISTANCE_THRESHOLD_METERS;
    for (const Vehicle &v : behind)
    {double dist = distance(ego.x, ego.y, v.x, v.y);
        if (dist < min_distance)
        { min_distance = dist;}}

    double diff_behind = (LANE_CHANGE_VEHICLE_BEHIND_DISTANCE_THRESHOLD_METERS - min_distance) / LANE_CHANGE_VEHICLE_BEHIND_DISTANCE_THRESHOLD_METERS;
    double diff = diff_behind + diff_ahead;
    return weight * (1 - exp(-diff));}
/////////////////////////////////////////////////////////////////////////////
double averageLaneSpeedDiffCostFunction(const Vehicle &ego, const vector<Vehicle>
&others, const Trajectory &trajectory,const State &state, const double &weight)
{  // We are switching lanes and this is not a cancellable operation
    if (state.d_state == LateralState::CHANGE_LANE_LEFT || state.d_state == 
LateralState::CHANGE_LANE_RIGHT)
    {return 0;}
    // Not in lane so doesn't count
    if (!ego.isInLane)
    {return 0.0;}

    // Find all vehicles ahead
    vector<Vehicle> ahead = ego.ahead(others, state.future_lane);
    if (ahead.size() == 0)
    { return 0.0;}
    double speed_avg = 0.0;    
    int count = 0;
    for (const Vehicle &v : ahead)
    { double dist = distance(ego.x, ego.y, v.x, v.y);
      // Only look a bit ahead
        if(dist <= VEHICLE_DISTANCE_THRESHOLD_METERS * 1.5)
        {speed_avg += v.getSpeed();
            ++count; }}
     if(count == 0)
    {return 0.0;}
    speed_avg /= (double)count;
    cout << "** Speed average of lane " << state.future_lane << ": " << speed_avg << endl;
   
    if (speed_avg >= MAX_SPEED_METERS_PER_SECOND)
    { return 0.0;
    }
    double diff = (MAX_SPEED_METERS_PER_SECOND - speed_avg)
 / MAX_SPEED_METERS_PER_SECOND;
    return weight * (1 - exp(-abs(diff)));}
////////////////////////////////////////////////////////////////////////////
double speedDifferenceWithClosestCarAheadCostFunction(const Vehicle &ego, 
const vector<Vehicle> &others, const Trajectory &trajectory,
const State &state, const double &weight)
{
    if (!ego.isInLane)
    {return 0.0;}
    vector<Vehicle> ahead = ego.ahead(others, state.current_lane);
    if (ahead.size() == 0){return 0.0;}
    double min_distance = VEHICLE_DISTANCE_THRESHOLD_METERS;
    Vehicle closest_vehicle;
    for (const Vehicle &v : ahead){if (v.s > ego.s)
        {double dist = distance(ego.x, ego.y, v.x, v.y);
         if (dist < min_distance){
                min_distance = dist;
                closest_vehicle = v;}}}

    if (min_distance >= VEHICLE_DISTANCE_THRESHOLD_METERS)
    {// No need to penalize if vehicle ahead is far enough...
        return 0.0;}
    double ego_speed = trajectory.averageSpeed(1.0);
    double v_speed = closest_vehicle.getSpeed();
   if (ego_speed > v_speed)
    {return weight;}
    double diff = v_speed - ego_speed;
    return weight * (1 - exp(-abs(diff)));}
//////////////////////////////////////////////////////////////////////
double lanesAverageForwardSpeedCarsAhead(const Vehicle &ego, 
const vector<Vehicle> &others, const Trajectory &trajectory,
const State &state, const double &weight)
{   
    if (!ego.isInLane)
    {return weight;}
    double min_distance = VEHICLE_DISTANCE_THRESHOLD_METERS;
    Vehicle closest_vehicle;
    for (const Vehicle &v : others)
    {// Other car must be ahead in the same lane
        if (v.isInLane && v.lane == state.future_lane && v.s > ego.s)
        {double dist = distance(ego.x, ego.y, v.x, v.y);
            if (dist < min_distance){min_distance = dist;closest_vehicle = v;
            }}}

    if (min_distance >= VEHICLE_DISTANCE_THRESHOLD_METERS)
    {// No need to penalize if vehicle ahead is far enough...
        return 0.0;}
    double ego_speed = trajectory.averageSpeed(1.0);
    double v_speed = closest_vehicle.getSpeed();
    double diff = v_speed - ego_speed;
    return weight * (1 - exp(-abs(diff)));}

////////////////////////////////////////////////////////////////////
double collisionTimeCostFunction(const Vehicle &ego, 
const vector<Vehicle> &others, const Trajectory &trajectory,
const State &state, const double &weight)
{  if (!ego.isInLane)
    {return weight;}
    double min_distance = VEHICLE_DISTANCE_THRESHOLD_METERS;
    Vehicle closest_vehicle;
    for (const Vehicle &v : others)
    { if (v.isInLane && (v.lane == state.current_lane || 
v.lane == state.future_lane) && v.s >= ego.s)
        {double dist = distance(ego.x, ego.y, v.x, v.y);
            if (dist < min_distance){
                min_distance = dist;
                closest_vehicle = v;}}}

    if (min_distance >= VEHICLE_DISTANCE_THRESHOLD_METERS)
    {// No need to penalize if vehicle ahead is far enough...
        return 0.0;}
    CollisionDetector detector = CollisionDetector(trajectory);
    Collision collision = detector.predictCollision
(closest_vehicle, CONTROLLER_UPDATE_RATE_SECONDS);
    if (!collision.willCollide)
    {// If no collision foreseen then don't penalize
        return 0.0;}
    double ego_speed = trajectory.averageSpeed(1.0);
    
    if (collision.collision_timestep > COLLISION_MAX_TIMESTEP_THRESHOLD)
    {        return 0.0;    }
    double speed_ratio = ego_speed / MAX_SPEED_METERS_PER_SECOND;
    double timestep_ratio = (COLLISION_MAX_TIMESTEP_THRESHOLD - 
collision.collision_timestep) / COLLISION_MAX_TIMESTEP_THRESHOLD;
    cout << "*** SPEED RATIO = " << speed_ratio << endl;
    cout << "*** TIMESTEP RATIO = " << timestep_ratio << endl;
    // double diff = 75 - (collision.collision_timestep + 5 * speed_ratio);
    double diff = speed_ratio + timestep_ratio;
    cout << "*** TIMESTEP + SPEED RATIO = " << diff << endl;

    // Otherwise penalize as a factor of 
//the time to collision-the furtheraway in time the better
    return weight * (1 - exp(-abs(diff)));
}

//////////////////////////////////////////////////////////////////
double futureDistanceToGoalCostFunction(const Vehicle &ego,
const vector<Vehicle> &others, const Trajectory &trajectory,
const State &state, const double &weight)
{    int traj_size = trajectory.size();
    double diff = MAX_TRACK_S - trajectory.ss[traj_size - 1];  
    return weight * (1 - exp(-abs(diff / MAX_TRACK_S)));}
