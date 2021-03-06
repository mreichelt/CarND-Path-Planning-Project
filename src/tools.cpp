#include "tools.h"
#include "const.h"

#include <cmath>


double deg2rad(double deg) { return deg * M_PI / 180; }

double rad2deg(double rad) { return rad * 180 / M_PI; }

double mph2mps(double mph) {
  return mph * 0.44704;
}


double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

size_t ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y) {

  double closestLen = 100000; //large number
  size_t closestWaypoint = 0;

  for (size_t i = 0; i < maps_x.size(); i++) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x, y, map_x, map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }

  }

  return closestWaypoint;
}

size_t NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y) {

  size_t closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y - y), (map_x - x));

  double angle = abs(theta - heading);

  if (angle > M_PI / 4) {
    closestWaypoint++;
  }

  return closestWaypoint;
}

vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y) {
  size_t next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);

  size_t prev_wp;
  prev_wp = next_wp - 1;
  if (next_wp == 0) {
    prev_wp = maps_x.size() - 1;
  }

  double n_x = maps_x[next_wp] - maps_x[prev_wp];
  double n_y = maps_y[next_wp] - maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
  double proj_x = proj_norm * n_x;
  double proj_y = proj_norm * n_y;

  double frenet_d = distance(x_x, x_y, proj_x, proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000 - maps_x[prev_wp];
  double center_y = 2000 - maps_y[prev_wp];
  double centerToPos = distance(center_x, center_y, x_x, x_y);
  double centerToRef = distance(center_x, center_y, proj_x, proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; i++) {
    frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
  }

  frenet_s += distance(0, 0, proj_x, proj_y);

  return {frenet_s, frenet_d};
}

vector<double>
getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp + 1] && (prev_wp < (int) (maps_s.size() - 1))) {
    prev_wp++;
  }

  size_t wp2 = (prev_wp + 1) % maps_x.size();

  double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s - maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
  double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

  double perp_heading = heading - M_PI / 2;

  double x = seg_x + d * cos(perp_heading);
  double y = seg_y + d * sin(perp_heading);

  return {x, y};
}

void transform_coordinates(vector<double> &x_list, vector<double> &y_list, const vector<double> &origin, double theta) {
  for (size_t i = 0; i < x_list.size(); i++) {
    // translate
    x_list[i] -= origin[0];
    y_list[i] -= origin[1];

    // rotate
    const double
      x = x_list[i],
      y = y_list[i];
    x_list[i] = x * cos(theta) + y * sin(theta);
    y_list[i] = -x * sin(theta) + y * cos(theta);
  }
}

void reverse_transform_coordinates(vector<double> &x_list, vector<double> &y_list, const vector<double> &origin,
                                   double theta) {
  for (size_t i = 0; i < x_list.size(); i++) {
    // rotate back
    const double
      x = x_list[i],
      y = y_list[i];
    x_list[i] = x * cos(-theta) + y * sin(-theta);
    y_list[i] = -x * sin(-theta) + y * cos(-theta);

    // translate back
    x_list[i] += origin[0];
    y_list[i] += origin[1];
  }
}

double clip(double d, double lower, double upper) {
  return max(lower, min(d, upper));
}

double getFollowerSpeed(double speed, double distance) {
  double max_distance = CORRIDOR_DISTANCE;
  double distance_desired = OPTIMUM_DISTANCE;
  if (distance >= max_distance) {
    return MAX_SPEED;
  }

  // generate a line f(x) = m*x + n
  double m = (MAX_SPEED - speed) / (max_distance - distance_desired);
  double n = MAX_SPEED - m * max_distance;
  double follower_speed = m * distance + n;

  return clip(follower_speed, MIN_SPEED, MAX_SPEED);
}

template<typename T>
int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

double getVelocityChange(double current_speed, double desired_speed) {
  double diff = desired_speed - current_speed;
  if (fabs(diff) > 1.0) {
    return sgn(diff) * MAX_SPEED_CHANGE;
  }
  return diff * MAX_SPEED_CHANGE;
}

double s_distance(double my_s, double other_s) {
  double diff = other_s - my_s;
  if (diff < -MAX_S / 2) {
    return diff + MAX_S;
  }
  if (diff > MAX_S / 2) {
    return diff - MAX_S;
  }
  return diff;
}
