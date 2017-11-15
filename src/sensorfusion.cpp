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

SensorFusion::SensorFusion(const vector<vector<double>> &sensorFusionList, double s, double delta_t)
  : vehicles(getValidVehicles(sensorFusionList)), s(s), delta_t(delta_t) {
  for (int lane = 0; lane < LANES; lane++) {
    nextVehicleInfos.push_back(getNextVehicleInfo(lane));
  }
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

double SensorFusion::getMinimalSpeedInFrontOf(int lane) {
  double minimalSpeed = MAX_SPEED;
  for (Vehicle vehicle : getVehicles(lane)) {
    if (vehicle.getPredictedS(delta_t) > (s - 10) && vehicle.getSpeed() < minimalSpeed) {
      minimalSpeed = vehicle.getSpeed();
    }
  }
  return minimalSpeed;
}

NextVehicleInfo SensorFusion::getNextVehicleInfo(int lane) {
  NextVehicleInfo info{false, MAX_ROOM, MAX_SPEED};

  vector<Vehicle> vehiclesInFront;
  double &s_ref = s, &delta_t_ref = delta_t;
  std::copy_if(begin(vehicles), end(vehicles), std::back_inserter(vehiclesInFront),
               [&lane, &s_ref, &delta_t_ref](Vehicle &vehicle) {
                   double distance = s_distance(s_ref, vehicle.getPredictedS(delta_t_ref));
                   return vehicle.isInLane(lane) && distance >= 0.0;
               });

  double min_distance = numeric_limits<double>::max();
  for (Vehicle vehicleInFront : vehiclesInFront) {
    double distance = s_distance(s, vehicleInFront.getPredictedS(delta_t));
    if (distance < min_distance) {
      min_distance = distance;
      info.hasNextVehicle = true;
      info.distance = distance;
      info.speed = vehicleInFront.getSpeed();
    }
  }

  return info;
}
