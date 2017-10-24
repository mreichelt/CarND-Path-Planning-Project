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

TEST_CASE("parse id correctly when int is not correctly parsed due to floating point issues") {
  Vehicle vehicle((vector<double>) {5.9999999999999,
                                    910.4965,
                                    1124.818,
                                    22.2936,
                                    0.121575,
                                    125.8168,
                                    10.02528});

  REQUIRE(vehicle.id == 6);
}

TEST_CASE("parse all vehicles from sensor fusion data") {
  vector<vector<double>> sensorFusionList = {
    {0,
      894.6071,
      1128.782,
      20.90929,
      -0.01702391,
      110.0156,
      6.019912},
    {1,
      910.4965,
      1124.818,
      22.2936,
      0.121575,
      125.8168,
      10.02528
    }
  };

  vector<Vehicle> vehicles = getValidVehicles(sensorFusionList);
  REQUIRE(vehicles.size() == 2);
  REQUIRE(vehicles[0].id == 0);
  REQUIRE(vehicles[1].id == 1);
}

TEST_CASE("filter only valid vehicles") {
  vector<vector<double>> sensorFusionList = {
    {0,
      0,
      0,
      0,
      0,
      0,
      6.019912},
    {1,
      0,
      0,
      0,
      0,
      0,
      -200.02528
    }
  };

  vector<Vehicle> vehicles = getValidVehicles(sensorFusionList);
  REQUIRE(vehicles.size() == 1);
  REQUIRE(vehicles[0].id == 0);
}

TEST_CASE("vehicle with very low d should be invalid") {
  Vehicle vehicle(0, 0, 0, 0, 0, 0, -200);
  REQUIRE_FALSE(vehicle.isValid());
}

TEST_CASE("vehicle with high d should be invalid") {
  Vehicle vehicle(0, 0, 0, 0, 0, 0, 20);
  REQUIRE_FALSE(vehicle.isValid());
}

TEST_CASE("vehicle with regular d should be valid") {
  Vehicle vehicle(0, 0, 0, 0, 0, 0, 8);
  REQUIRE(vehicle.isValid());
}
