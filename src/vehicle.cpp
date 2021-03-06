#include "vehicle.h"
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

double Vehicle::getSpeed() {
  return sqrt(vx * vx + vy * vy);
}

bool Vehicle::isInLane(int lane) {
  double lane_start = LANE_WIDTH * lane;
  double lane_end = lane_start + LANE_WIDTH;
  return d > lane_start && d <= lane_end;
}

double Vehicle::getPredictedS(double delta_t) {
  return s + getSpeed() * delta_t;
}
