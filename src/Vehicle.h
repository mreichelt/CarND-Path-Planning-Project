#ifndef PATH_PLANNING_VEHICLE_H
#define PATH_PLANNING_VEHICLE_H

#include <vector>

using namespace std;

class Vehicle {
public:
    const int id;
    const double x;
    const double y;
    const double vx;
    const double vy;
    const double s;
    const double d;

    Vehicle(int id, double x, double y, double vx, double vy, double s, double d);

    explicit Vehicle(const vector<double> &sensorFusion);

    bool isValid();

};

vector<Vehicle> getValidVehicles(vector<vector<double>> &sensorFusionList);

#endif //PATH_PLANNING_VEHICLE_H
