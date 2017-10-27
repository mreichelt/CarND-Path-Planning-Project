#ifndef PATH_PLANNING_CONST_H
#define PATH_PLANNING_CONST_H

#include "tools.h"

const int LANES = 3;
const double LANE_WIDTH = 4.0;
const double MIN_D = 0.0;
const double MAX_D = LANES * LANE_WIDTH;
const double MAX_SPEED = mph2mps(49.5);
const double T = 0.02;
const int N_WAYPOINTS = 50;

#endif //PATH_PLANNING_CONST_H
