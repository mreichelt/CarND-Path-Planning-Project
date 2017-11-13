#include "planning.h"
#include <map>
#include <iostream>

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
  double min_lane_speed = args.sensorFusion.getMinimalSpeed(args.proposed_lane);
  double cost = (MAX_SPEED - min_lane_speed) / MAX_SPEED;
  return max(0.0, cost);
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
  double s_corridor_start = args.s - COLLISION_AVOIDANCE_RANGE;
  double s_corridor_end = args.s + COLLISION_AVOIDANCE_RANGE;
  for (Vehicle vehicleInProposedLane : vehiclesInProposedLane) {
    double predictedS = vehicleInProposedLane.getPredictedS(args.delta_t);
    if (predictedS >= s_corridor_start && predictedS <= s_corridor_end) {
      return 1.0;
    }
  }

  return 0.0;
}

double cost(CostFunctionArgs &args) {
  map<CostFunction, double> costFunctions = {
    {cost_change_lane,  WEIGHT_COMFORT},
    {cost_inefficiency, WEIGHT_EFFICIENCY},
    {cost_collision,    WEIGHT_COLLISION}
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
  if (targetLane > 0) {
    possibleStates.push_back(STATE_CHANGE_LEFT);
  }
  possibleStates.push_back(STATE_KEEP_LANE);
  if (targetLane < LANES) {
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
int next_state(SensorFusion sensorFusion, int targetLane, double s, double d, double delta_t) {
  vector<int> possibleStates = getPossibleStates(targetLane);
  vector<double> costs;
  for (int possibleState : possibleStates) {
    CostFunctionArgs args(possibleState, sensorFusion, targetLane, s, d, delta_t);
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
                                   double d,
                                   double delta_t)
  : state(state),
    sensorFusion(sensorFusion),
    current_lane(current_lane),
    proposed_lane(getNewLane(state, current_lane)),
    s(s),
    d(d),
    delta_t(delta_t) {}

int getNewLane(int state, int current_lane) {
  switch (state) {
    case STATE_CHANGE_LEFT:
      return max(0, current_lane - 1);

    case STATE_CHANGE_RIGHT:
      return min(LANES, current_lane + 1);

    default:
      return current_lane;
  }
}
