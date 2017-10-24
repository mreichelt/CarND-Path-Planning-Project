#include "Vehicle.h"
#include "const.h"
#include <cmath>

Vehicle::Vehicle(int id, double x, double y, double vx, double vy, double s, double d)
  : id(id), x(x), y(y), vx(vx), vy(vy), s(s), d(d) {}

Vehicle::Vehicle(const std::vector<double> &sensorFusion)
  : id((int) std::round(sensorFusion[0])),
    x(sensorFusion[1]),
    y(sensorFusion[2]),
    vx(sensorFusion[3]),
    vy(sensorFusion[4]),
    s(sensorFusion[5]),
    d(sensorFusion[6]) {}

bool Vehicle::isValid() {
  return MIN_D < this->d && this->d < MAX_D;
}

vector<Vehicle> getValidVehicles(vector<vector<double>> &sensorFusionList) {
  vector<Vehicle> vehicles;
  for (const vector<double> &sensorFusion : sensorFusionList) {
    Vehicle vehicle(sensorFusion);
    if (vehicle.isValid()) {
      vehicles.push_back(vehicle);
    }
  }
  return vehicles;
}
