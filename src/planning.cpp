#include "planning.h"
#include <map>
#include <iostream>

/**
 * Keeping the current lane in general is more comfortable than changing lanes.
 */
double cost_change_lane(const CostFunctionArgs &args) {
  return args.state == STATE_KEEP_LANE ? 0.0 : 1.0;
}

double cost_inefficiency(const CostFunctionArgs &args) {
  // TODO
  return 0.0;
}

double cost_collision(const CostFunctionArgs &args) {
  // TODO
  return 0.0;
}

double cost(const CostFunctionArgs &args) {
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
int next_state(SensorFusion sensorFusion, int targetLane, double s, double d) {
  vector<int> possibleStates = getPossibleStates(targetLane);
  vector<double> costs;
  for (int possibleState : possibleStates) {
    CostFunctionArgs args(possibleState, sensorFusion, targetLane, s, d);
    costs.push_back(cost(args));
  }
  int state = possibleStates[getMinIndex(costs)];

  cout << "Next state is " << state << endl;
  return state;
}

CostFunctionArgs::CostFunctionArgs(int state,
                                   SensorFusion sensorFusion,
                                   int target_lane,
                                   double s,
                                   double d)
  : state(state),
    sensorFusion(sensorFusion),
    target_lane(target_lane),
    s(s),
    d(d) {}
