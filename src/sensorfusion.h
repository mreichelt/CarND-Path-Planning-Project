#ifndef PATH_PLANNING_SENSORFUSION_H
#define PATH_PLANNING_SENSORFUSION_H

#include "vehicle.h"

class SensorFusion {
public:
    const vector<Vehicle> vehicles;

    explicit SensorFusion(const vector<vector<double>> &sensorFusionList);
};

#endif //PATH_PLANNING_SENSORFUSION_H
