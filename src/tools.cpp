#include "tools.h"

#include <cmath>

double deg2rad(double deg) { return deg * M_PI / 180; }

double rad2deg(double rad) { return rad * 180 / M_PI; }

double mph2mps(double mph) {
  return mph * 0.44704;
}
