#include "sensorfusion.h"


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
