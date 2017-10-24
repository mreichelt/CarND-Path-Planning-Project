#ifndef PATH_PLANNING_VEHICLE_H
#define PATH_PLANNING_VEHICLE_H

#include <vector>

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

    explicit Vehicle(const std::vector<double> &sensorFusion);

};


#endif //PATH_PLANNING_VEHICLE_H
