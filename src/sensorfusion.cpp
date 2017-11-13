#include "sensorfusion.h"
#include "const.h"


vector<Vehicle> getValidVehicles(const vector<vector<double>> &sensorFusionList) {
  vector<Vehicle> vehicles;
  for (const vector<double> &sensorFusion : sensorFusionList) {
    Vehicle vehicle(sensorFusion);
    if (vehicle.isValid()) {
      vehicles.push_back(vehicle);
    }
  }
  return vehicles;
}

SensorFusion::SensorFusion(const vector<vector<double>> &sensorFusionList)
  : vehicles(getValidVehicles(sensorFusionList)) {
}


vector<Vehicle> SensorFusion::getVehicles(int lane) {
  vector<Vehicle> vehiclesInLane;
  for (Vehicle &vehicle : vehicles) {
    if (vehicle.isInLane(lane)) {
      vehiclesInLane.push_back(vehicle);
    }
  }
  return vehiclesInLane;
}

double SensorFusion::getMinimalSpeed(int lane) {
  double minimalSpeed = MAX_SPEED;
  for (Vehicle vehicle : getVehicles(lane)) {
    if (vehicle.getSpeed() < minimalSpeed) {
      minimalSpeed = vehicle.getSpeed();
    }
  }
  return minimalSpeed;
}
