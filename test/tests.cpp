#define CATCH_CONFIG_MAIN

#include "catch.hpp"
#include "../src/Vehicle.h"

using namespace std;

TEST_CASE("parse basic sensor fusion vector") {
  Vehicle vehicle((vector<double>) {6,
                                    910.4965,
                                    1124.818,
                                    22.2936,
                                    0.121575,
                                    125.8168,
                                    10.02528});

  REQUIRE(vehicle.id == 6);
  REQUIRE(vehicle.x == Approx(910.4965));
  REQUIRE(vehicle.y == Approx(1124.818));
  REQUIRE(vehicle.vx == Approx(22.2936));
  REQUIRE(vehicle.vy == Approx(0.121575));
  REQUIRE(vehicle.s == Approx(125.8168));
  REQUIRE(vehicle.d == Approx(10.02528));
}

TEST_CASE("parse id correctly when int is not correctly parsed by JSON") {
  Vehicle vehicle((vector<double>) {5.9999999999999,
                                    910.4965,
                                    1124.818,
                                    22.2936,
                                    0.121575,
                                    125.8168,
                                    10.02528});

  REQUIRE(vehicle.id == 6);
}
