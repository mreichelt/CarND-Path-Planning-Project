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
    double s;
    double delta_t;
    vector<NextVehicleInfo> nextVehicleInfos;

    explicit SensorFusion(const vector<vector<double>> &sensorFusionList, double s, double delta_t);

    vector<Vehicle> getVehicles(int lane);

    NextVehicleInfo getNextVehicleInfo(int lane);

    double getMinimalSpeedInFrontOf(int lane);
};

#endif //PATH_PLANNING_SENSORFUSION_H
