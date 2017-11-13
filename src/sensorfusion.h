#ifndef PATH_PLANNING_SENSORFUSION_H
#define PATH_PLANNING_SENSORFUSION_H

#include "vehicle.h"

class SensorFusion {
public:
    vector<Vehicle> vehicles;

    explicit SensorFusion(const vector<vector<double>> &sensorFusionList);

    vector<Vehicle> getVehicles(int lane);

    double getMinimalSpeed(int lane);
};

#endif //PATH_PLANNING_SENSORFUSION_H
