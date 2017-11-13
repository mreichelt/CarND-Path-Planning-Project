#ifndef PATH_PLANNING_PLANNING_H
#define PATH_PLANNING_PLANNING_H

#include "vehicle.h"
#include "sensorfusion.h"
#include "const.h"

static int const STATE_KEEP_LANE = 0;
static int const STATE_CHANGE_LEFT = 1;
static int const STATE_CHANGE_RIGHT = 2;

static const double WEIGHT_COLLISION = 10e6;
static const double WEIGHT_EFFICIENCY = 10e4;
static const double WEIGHT_COMFORT = 10e2;

class CostFunctionArgs {
public:
    int state;
    SensorFusion sensorFusion;

    int current_lane;
    int proposed_lane;

    double s;
    double d;
    double delta_t;

    CostFunctionArgs(int state, SensorFusion sensorFusion, int current_lane, double s, double d, double delta_t);
};

typedef double (*CostFunction)(CostFunctionArgs &args);

double cost(CostFunctionArgs &args);

int next_state(SensorFusion sensorFusion, int targetLane, double s, double d, double delta_t);

int getNewLane(int state, int current_lane);

#endif //PATH_PLANNING_PLANNING_H
