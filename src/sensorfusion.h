#ifndef PATH_PLANNING_SENSORFUSION_H
#define PATH_PLANNING_SENSORFUSION_H

#include "vehicle.h"
#include "const.h"

struct NextVehicleInfo {
    bool hasNextVehicle;
    double distance;
    double speed;
};

class SensorFusion {
public:
    vector<Vehicle> vehicles;

    explicit SensorFusion(const vector<vector<double>> &sensorFusionList);

    vector<Vehicle> getVehicles(int lane);

    NextVehicleInfo getNextVehicleInfo(int lane, double s, double delta_t);

    double getMinimalSpeedInFrontOf(int lane, double s, double delta_t);
};

#endif //PATH_PLANNING_SENSORFUSION_H
