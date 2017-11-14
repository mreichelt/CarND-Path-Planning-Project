#define CATCH_CONFIG_MAIN

#include "catch.hpp"
#include "../src/vehicle.h"
#include "../src/sensorfusion.h"
#include "../src/tools.h"

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

  SensorFusion sensorFusion(sensorFusionList);
  vector<Vehicle> vehicles = sensorFusion.vehicles;
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

  SensorFusion sensorFusion(sensorFusionList);
  vector<Vehicle> vehicles = sensorFusion.vehicles;
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

TEST_CASE("convert 50 miles/hour to meters/second") {
  REQUIRE(mph2mps(50) == Approx(22.352));
}

TEST_CASE("convert 2.24 miles/hour to meters/second") {
  REQUIRE(mph2mps(2.23694) == Approx(1));
}

TEST_CASE("convert -2.24 miles/hour to meters/second") {
  REQUIRE(mph2mps(-2.23694) == Approx(-1));
}

TEST_CASE("simple forward transformation") {
  vector<double>
    x = {1.0},
    y = {1.0};
  transform_coordinates(x, y, {1.0, 1.0}, 0.0);
  REQUIRE(x[0] == Approx(0.0));
  REQUIRE(y[0] == Approx(0.0));
}

TEST_CASE("simple forward transformation that includes rotation") {
  vector<double>
    x = {2.0},
    y = {2.0};
  transform_coordinates(x, y, {1.0, 1.0}, M_PI_4);
  REQUIRE(x[0] == Approx(sqrt(2)));
  REQUIRE(y[0] == Approx(0.0));
}

TEST_CASE("forward and reverse transformation") {
  vector<double>
    x = {2.0},
    y = {2.0};
  transform_coordinates(x, y, {1.0, 1.0}, M_PI_4);
  REQUIRE(x[0] == Approx(sqrt(2)));
  REQUIRE(y[0] == Approx(0.0));

  reverse_transform_coordinates(x, y, {1.0, 1.0}, M_PI_4);
  REQUIRE(x[0] == Approx(2.0));
  REQUIRE(y[0] == Approx(2.0));
}

void require_approx_same(const vector<double> &actual, const vector<double> &expected) {
  REQUIRE(actual.size() == expected.size());
  for (size_t i = 0; i < expected.size(); i++) {
    REQUIRE(actual[i] == Approx(expected[i]));
  }
}

TEST_CASE("forward and reverse transform multiple coordinates") {
  vector<double>
    x = {1.0, 1.0, 1.0},
    y = {1.0, 2.0, 3.0};
  transform_coordinates(x, y, {1.0, 1.0}, M_PI_2);
  require_approx_same(x, {0.0, 1.0, 2.0});
  require_approx_same(y, {0.0, 0.0, 0.0});

  reverse_transform_coordinates(x, y, {1.0, 1.0}, M_PI_2);
  require_approx_same(x, {1.0, 1.0, 1.0});
  require_approx_same(y, {1.0, 2.0, 3.0});
}

TEST_CASE("find vehicles in lane") {
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

  SensorFusion sensorFusion(sensorFusionList);
  REQUIRE(sensorFusion.getVehicles(0).size() == 0);
  REQUIRE(sensorFusion.getVehicles(1).size() == 1);
  REQUIRE(sensorFusion.getVehicles(2).size() == 1);
}

TEST_CASE("follower speed should be max speed for more than 100m distance") {
  REQUIRE(getFollowerSpeed(mph2mps(40.0), 105.0) == Approx(MAX_SPEED));
}

TEST_CASE("follower speed should be at exactly speed when distance is 40m") {
  REQUIRE(getFollowerSpeed(mph2mps(30.0), 40.0) == Approx(mph2mps(30.0)));
}

TEST_CASE("follower speed should be smaller then speed of other vehicle when too close") {
  REQUIRE(getFollowerSpeed(mph2mps(30.0), 20.0) < mph2mps(25.0));
}

TEST_CASE("follower speed should not be negative") {
  REQUIRE(getFollowerSpeed(mph2mps(1.0), 0.0) >= 0.0);
}

TEST_CASE("desired speed change is max") {
  // accelerate
  REQUIRE(getVelocityChange(mph2mps(0.0), MAX_SPEED) == Approx(MAX_SPEED_CHANGE));
  REQUIRE(getVelocityChange(mph2mps(0.0), 2 * MAX_SPEED) == Approx(MAX_SPEED_CHANGE));

  // decelerate
  REQUIRE(getVelocityChange(MAX_SPEED, mph2mps(0.0)) == Approx(-MAX_SPEED_CHANGE));
  REQUIRE(getVelocityChange(2 * MAX_SPEED, mph2mps(0.0)) == Approx(-MAX_SPEED_CHANGE));
}

TEST_CASE("desired speed change is 0") {
  REQUIRE(getVelocityChange(mph2mps(25.0), mph2mps(25.0)) == Approx(0.0));
}
