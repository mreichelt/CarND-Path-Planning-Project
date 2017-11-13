#ifndef PATH_PLANNING_CONST_H
#define PATH_PLANNING_CONST_H

#include "tools.h"
#include <chrono>

const int LANES = 3;
const int MIN_LANE = 0;
const int MAX_LANE = LANES - 1;
const double LANE_WIDTH = 4.0;
const double MIN_D = 0.0;
const double MAX_D = LANES * LANE_WIDTH;
const double MIN_SPEED = mph2mps(0.001);
const double MAX_SPEED = mph2mps(49.5);
const double T = 0.02;
const int N_WAYPOINTS = 50;
const auto PLANNING_INTERVAL = chrono::seconds(3);
const double COLLISION_AVOIDANCE_RANGE = 40.0;

#endif //PATH_PLANNING_CONST_H
