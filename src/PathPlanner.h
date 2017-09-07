#pragma once
#include <vector>
using namespace std;

void planPath(vector <vector <double> > & sensor_fusion,
	double car_x, double car_y, double car_yaw, double car_s, double car_d, double car_speed,
	vector<double>& next_x_vals, vector<double>& next_y_vals,
	const vector<double>& previous_x_vals, const vector<double>& previous_y_vals,
	double end_path_s);