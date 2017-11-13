#include <fstream>
#include <cmath>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "tools.h"
#include "const.h"
#include "vehicle.h"
#include "sensorfusion.h"
#include "planning.h"

using namespace std;
using json = nlohmann::json;
using system_clock = chrono::system_clock;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(const string &s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of('[');
  auto b2 = s.find_first_of('}');
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
    double x;
    double y;
    double s;
    double d_x;
    double d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  int target_lane = 1;
  auto next_planning = system_clock::now() + 2 * PLANNING_INTERVAL;

  h.onMessage([&target_lane, &next_planning,
                &map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy](
    uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
    uWS::OpCode opCode) {
      // "42" at the start of the message means there's a websocket message event.
      // The 4 signifies a websocket message
      // The 2 signifies a websocket event
      //auto sdata = string(data).substr(0, length);
      //cout << sdata << endl;
      if (length > 2 && data[0] == '4' && data[1] == '2') {

        auto jsonString = hasData(data);

        if (!jsonString.empty()) {
          auto j = json::parse(jsonString);

          string event = j[0].get<string>();

          if (event == "telemetry") {
            double car_x = j[1]["x"];
            double car_y = j[1]["y"];
            double car_s = j[1]["s"];
            double car_d = j[1]["d"];
            double car_yaw = deg2rad(j[1]["yaw"]);
            double car_speed = mph2mps(j[1]["speed"]);
            vector<double> previous_path_x = j[1]["previous_path_x"];
            vector<double> previous_path_y = j[1]["previous_path_y"];
            double end_path_s = j[1]["end_path_s"];
            double end_path_d = j[1]["end_path_d"];
            vector<vector<double>> raw_sensor_fusion = j[1]["sensor_fusion"];

            // convert raw sensor fusion data to something a little bit improved
            SensorFusion sensorFusion(raw_sensor_fusion);

            // check if we need to plan again
            int state = STATE_KEEP_LANE;
            if (system_clock::now() > next_planning) {
              next_planning = system_clock::now() + PLANNING_INTERVAL;
              cout << "Planning" << endl;
              state = next_state(sensorFusion, target_lane, end_path_s, end_path_d);
            }

            // apply new state
            if (state == STATE_CHANGE_LEFT && target_lane > 0) {
              target_lane--;
            }
            if (state == STATE_CHANGE_RIGHT && target_lane < LANES) {
              target_lane++;
            }

            vector<double> ptsx, ptsy;

            size_t previousPathSize = previous_path_x.size();

            bool too_close = false;
            double target_d = 2 + target_lane * 4;

            if (previousPathSize > 0) {
              car_s = end_path_s;
              car_d = end_path_d;
            }

            for (Vehicle vehicle : sensorFusion.getVehicles(target_lane)) {
              double predicted_s = vehicle.getPredictedS(previousPathSize * T);
              if (predicted_s > car_s && (predicted_s - car_s) < 30.0) {
                too_close = true;
              }
            }


            // generate initial 2 waypoints
            double x1, y1, ref_x, ref_y;
            double ref_yaw = car_yaw;
            double end_car_speed = car_speed;
            if (previousPathSize < 2) {
              x1 = car_x - cos(car_yaw);
              y1 = car_y - sin(car_yaw);
              ref_x = car_x;
              ref_y = car_y;
            } else {
              x1 = previous_path_x[previousPathSize - 2];
              y1 = previous_path_y[previousPathSize - 2];
              ref_x = previous_path_x[previousPathSize - 1];
              ref_y = previous_path_y[previousPathSize - 1];
              ref_yaw = atan2(ref_y - y1, ref_x - x1);
              double dist = distance(x1, y1, ref_x, ref_y);
              end_car_speed = dist / T;
            }
            ptsx.push_back(x1);
            ptsx.push_back(ref_x);
            ptsy.push_back(y1);
            ptsy.push_back(ref_y);

            double ref_speed = end_car_speed;
            double velocity_change = mph2mps(0.3);
            if (too_close) {
              ref_speed -= velocity_change;
            } else {
              ref_speed += velocity_change;
            }
            ref_speed = clip(ref_speed, MIN_SPEED, MAX_SPEED);

            // generate sparse target waypoints
            double meters = 15.0;
            double current_d = car_d;
            int steps = 2;
            double d_delta = (target_d - car_d) / steps;
            for (int i = 1; i <= steps; i++) {
              vector<double> xy = getXY(car_s + meters * i, current_d + (i * d_delta),
                                        map_waypoints_s, map_waypoints_x, map_waypoints_y);
              ptsx.push_back(xy[0]);
              ptsy.push_back(xy[1]);
            }

            // place origin into the car's location so we can work with spline
            transform_coordinates(ptsx, ptsy, {ref_x, ref_y}, ref_yaw);

            tk::spline s;
            s.set_points(ptsx, ptsy);

            vector<double>
              next_x_vals(previous_path_x),
              next_y_vals(previous_path_y),
              additional_x,
              additional_y;

            double target_x = 30.0;
            double target_y = s(target_x);
            double target_dist = sqrt(target_x * target_x + target_y * target_y);
            double x = 0;
            double n = target_dist / (T * ref_speed);
            double x_step = target_x / n;

            for (int i = 0; i < N_WAYPOINTS - previousPathSize; i++) {
              x += x_step;
              additional_x.push_back(x);
              additional_y.push_back(s(x));
            }

            // restore original origin and rotation
            reverse_transform_coordinates(additional_x, additional_y, {ref_x, ref_y}, ref_yaw);

            // add additional waypoints to next waypoints
            next_x_vals.insert(next_x_vals.end(), additional_x.begin(), additional_x.end());
            next_y_vals.insert(next_y_vals.end(), additional_y.begin(), additional_y.end());

            // build and send JSON message to simulator
            json msgJson;
            msgJson["next_x"] = next_x_vals;
            msgJson["next_y"] = next_y_vals;
            string msg = "42[\"control\"," + msgJson.dump() + "]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
        } else {
          // Manual driving
          string msg = "42[\"manual\",{}]";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
      const string s = "<h1>Hello world!</h1>";
      if (req.getUrl().valueLength == 1) {
        res->end(s.data(), s.length());
      } else {
        // i guess this should be done more gracefully?
        res->end(nullptr, 0);
      }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
      cout << "Connected!!!" << endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
      ws.close();
      cout << "Disconnected" << endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    cout << "Listening to port " << port << endl;
  } else {
    cerr << "Failed to listen to port" << endl;
    return -1;
  }
  h.run();
}
