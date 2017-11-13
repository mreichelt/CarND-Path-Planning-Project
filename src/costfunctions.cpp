#include "costfunctions.h"
#include <map>
#include <iostream>

double cost_change_lane(const CostFunctionArgs &args) {
  // TODO
  return 0.0;
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

int next_state(SensorFusion sensorFusion, int targetLane, double s, double d) {
  int state = STATE_KEEP_LANE;
  if (targetLane == 1 && s > 200) {
    state = STATE_CHANGE_LEFT;
  }
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
