#ifndef PATH_PLANNING_TOOLS_H
#define PATH_PLANNING_TOOLS_H

#include <vector>

using namespace std;

double deg2rad(double deg);

double rad2deg(double rad);

/**
 * Converts miles/hour to meters/second.
 */
double mph2mps(double mph);

double distance(double x1, double y1, double x2, double y2);

size_t ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);

size_t NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);


// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double>
getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

/**
 * Transform a list of waypoints (given by x_list, y_list) so the new origin is at {@code origin} with an angle of 0.
 */
void transform_coordinates(vector<double> &x_list, vector<double> &y_list, const vector<double> &origin, double theta);

/**
 * Restore a list of waypoints that have been transformed into another coordinate system.
 */
void reverse_transform_coordinates(vector<double> &x_list, vector<double> &y_list, const vector<double> &origin,
                                   double theta);


#endif //PATH_PLANNING_TOOLS_H
