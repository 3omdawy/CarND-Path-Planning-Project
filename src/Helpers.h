#pragma once
#include <vector>
using namespace std;

// For converting back and forth between radians and degrees.
extern double deg2rad(double x);
extern double rad2deg(double x);
extern constexpr double pi();

extern void fillWayPoints(vector<double>& map_waypoints_x, vector<double>& map_waypoints_y, vector<double>& map_waypoints_s);
extern vector<double> getFrenet(double x, double y, double theta);
vector<double> getXY(double s, double d);