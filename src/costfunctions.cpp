#include "costfunctions.h"
#include <map>

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
