#include "planning.h"
#include <map>
#include <iostream>
#include <algorithm>
#include <cmath>

using namespace std;

/**
 * Keeping the current lane in general is more comfortable than changing lanes.
 */
double cost_change_lane(CostFunctionArgs &args) {
  return args.state == STATE_KEEP_LANE ? 0.0 : 1.0;
}

/**
 * A lane with higher possible speed should be preferred to a lane with lower speed.
 */
double cost_inefficiency(CostFunctionArgs &args) {
  double min_lane_speed = args.sensorFusion.getMinimalSpeedInFrontOf(args.proposed_lane);
  double cost = (MAX_SPEED - min_lane_speed) / MAX_SPEED;
  return max(0.0, cost);
}

/**
 * A lane where the next vehicle is further away should be preferred.
 */
double cost_room_for_driving(CostFunctionArgs &args) {
  NextVehicleInfo info = args.sensorFusion.getNextVehicleInfo(args.proposed_lane);
  if (!info.hasNextVehicle) {
    // no vehicle here - yeah, let's take it!
    return 0.0;
  }

  double cost = (MAX_ROOM - info.distance) / MAX_ROOM;
  return clip(cost, 0.0, 1.0);
}

/**
 * State changes that lead to collisions should be avoided.
 */
double cost_collision(CostFunctionArgs &args) {
  if (args.state == STATE_KEEP_LANE) {
    // staying on the current lane is considered safe, at least in the simulator
    return 0.0;
  }

  // otherwise, check if there would be a collision
  vector<Vehicle> vehiclesInProposedLane = args.sensorFusion.getVehicles(args.proposed_lane);

  // find vehicles that are pretty close to us
  vector<Vehicle> closeVehicles;
  std::copy_if(vehiclesInProposedLane.begin(), vehiclesInProposedLane.end(), std::back_inserter(closeVehicles),
               [&args](Vehicle &vehicle) {
                   return abs(s_distance(args.s, vehicle.getPredictedS(args.delta_t))) < NO_COLLISION_LIMIT;
               });

  for (Vehicle &closeVehicle : closeVehicles) {
    double distance = s_distance(args.s, closeVehicle.getPredictedS(args.delta_t));
    if (distance > DANGER_ZONE_FRONT || distance < -DANGER_ZONE_BACK) {
      return 1.0;
    }
    if (distance > 0) {
      // car is close to front, check if it is slowing down
      if (closeVehicle.getSpeed() < args.speed) {
        return 0.5;
      }
    } else {
      // car is close to rear, check if it is speeding up
      if (closeVehicle.getSpeed() > args.speed) {
        return 0.5;
      }
    }
  }

  // enough room for changing lanes
  return 0.0;
}

double cost(CostFunctionArgs &args) {
  map<CostFunction, double> costFunctions = {
    {cost_change_lane,      WEIGHT_COMFORT},
//    {cost_inefficiency,     WEIGHT_EFFICIENCY},
    {cost_room_for_driving, WEIGHT_EFFICIENCY},
    {cost_collision,        WEIGHT_COLLISION}
  };

  double cost = 0.0;
  for (auto &pair : costFunctions) {
    CostFunction costFunction = pair.first;
    double weight = pair.second;
    cost += weight * costFunction(args);
  }
  return cost;
}

/**
 * Only returns possible states for a current target lane - i.e. does a quick sanity check so we can not leave the
 * highway.
 */
vector<int> getPossibleStates(int targetLane) {
  vector<int> possibleStates;
  if (targetLane > MIN_LANE) {
    possibleStates.push_back(STATE_CHANGE_LEFT);
  }
  possibleStates.push_back(STATE_KEEP_LANE);
  if (targetLane < MAX_LANE) {
    possibleStates.push_back(STATE_CHANGE_RIGHT);
  }
  return possibleStates;
}

long getMinIndex(vector<double> values) {
  return distance(values.begin(), min_element(values.begin(), values.end()));
}

/**
 * Determines the next state to choose based on the current situation of the vehicle and its surroundings.
 */
int next_state(SensorFusion sensorFusion, int targetLane, double s, double speed, double delta_t) {
  cout << "next_state for lane = " << targetLane << endl;
  vector<int> possibleStates = getPossibleStates(targetLane);
  vector<double> costs;
  for (int possibleState : possibleStates) {
    CostFunctionArgs args(possibleState, sensorFusion, targetLane, s, speed, delta_t);
    costs.push_back(cost(args));
  }
  int state = possibleStates[getMinIndex(costs)];

  cout << "Next state is " << state << endl;

  return state;
}

CostFunctionArgs::CostFunctionArgs(int state,
                                   SensorFusion sensorFusion,
                                   int current_lane,
                                   double s,
                                   double speed,
                                   double delta_t)
  : state(state),
    sensorFusion(sensorFusion),
    current_lane(current_lane),
    proposed_lane(getNewLane(state, current_lane)),
    s(s),
    speed(speed),
    delta_t(delta_t) {}

int getNewLane(int state, int current_lane) {
  switch (state) {
    case STATE_CHANGE_LEFT:
      return current_lane - 1;

    case STATE_CHANGE_RIGHT:
      return current_lane + 1;

    default:
      return current_lane;
  }
}
