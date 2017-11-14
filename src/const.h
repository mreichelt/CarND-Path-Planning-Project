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
const double MAX_SPEED_CHANGE = mph2mps(0.3);
const double MAX_ROOM = 3000.0;
const double T = 0.02;
const int N_WAYPOINTS = 30;
const auto PLANNING_INTERVAL = chrono::seconds(3);
const double COLLISION_AVOIDANCE_FRONT = 40.0;
const double COLLISION_AVOIDANCE_BACK = 30.0;

const double OPTIMUM_DISTANCE = 30.0;
const double CORRIDOR_DISTANCE = 50.0;

#endif //PATH_PLANNING_CONST_H
