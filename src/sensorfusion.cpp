#include "sensorfusion.h"
#include "const.h"
#include <algorithm>


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

double SensorFusion::getMinimalSpeedInFrontOf(int lane, double s, double delta_t) {
  double minimalSpeed = MAX_SPEED;
  for (Vehicle vehicle : getVehicles(lane)) {
    if (vehicle.getPredictedS(delta_t) > s && vehicle.getSpeed() < minimalSpeed) {
      minimalSpeed = vehicle.getSpeed();
    }
  }
  return minimalSpeed;
}

NextVehicleInfo SensorFusion::getNextVehicleInfo(int lane, double s, double delta_t) {
  NextVehicleInfo info{false, MAX_ROOM, MAX_SPEED};

  vector<Vehicle> vehiclesInFront;
  std::copy_if(begin(vehicles), end(vehicles), std::back_inserter(vehiclesInFront),
               [&lane, &s, &delta_t](Vehicle &vehicle) {
                   return vehicle.isInLane(lane) && vehicle.getPredictedS(delta_t) > s;
               });

  double nearestS = numeric_limits<double>::max();
  for (Vehicle vehicleInFront : vehiclesInFront) {
    double predictedS = vehicleInFront.getPredictedS(delta_t);
    if (predictedS < nearestS) {
      info.hasNextVehicle = true;
      info.distance = predictedS - s;
      info.speed = vehicleInFront.getSpeed();
    }
  }

  return info;
}
