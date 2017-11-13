#ifndef PATH_PLANNING_COSTFUNCTIONS_H
#define PATH_PLANNING_COSTFUNCTIONS_H

#include "vehicle.h"
#include "sensorfusion.h"
#include "const.h"

static int const STATE_KEEP_LANE = 0;
static int const STATE_CHANGE_LEFT = 1;
static int const STATE_CHANGE_RIGHT = 2;

static const double WEIGHT_COLLISION = 10e6;
static const double WEIGHT_COMFORT = 10e4;
static const double WEIGHT_EFFICIENCY = 10e2;

class CostFunctionArgs {
public:
    int state;
    SensorFusion sensorFusion;

    int target_lane;

    double s;
    double d;

    CostFunctionArgs(int state, SensorFusion sensorFusion, int target_lane, double s, double d);
};

typedef double (*CostFunction)(const CostFunctionArgs &args);

double cost(const CostFunctionArgs &args);

int next_state(SensorFusion sensorFusion, int targetLane, double s, double d);

#endif //PATH_PLANNING_COSTFUNCTIONS_H
