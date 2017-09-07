#define _USE_MATH_DEFINES // Needed for include of math.h
#include <math.h>
#include "Helpers.h"
#include "Configurations.h"
#include "PathPlanner.h"
#include "spline.h"

void planPath(vector <vector <double> > & sensor_fusion,
	double car_x, double car_y, double car_yaw, double car_s, double car_d, double car_speed,
	vector<double>& next_x_vals, vector<double>& next_y_vals,
	const vector<double>& previous_x_vals, const vector<double>& previous_y_vals,
	double end_path_s)
{
	/*** Variables declaration ***/
	// Variable to hold current velocity (mph)
	static double ref_vel = 0;
	// Points to base the path on
	vector<double> ptsx, ptsy;
	// Reference x, y, and yaw for co-ordinates transformation between local and gloabal co-ordinates
	double ref_x = car_x;
	double ref_y = car_y;
	double ref_yaw = deg2rad(car_yaw);
	// Current ego lane
	// static: to be used when deciing to shift to a new lane
	static int lane = car_d / LANE_WIDTH;
	// Number of points that are still to be travelled based on previous path
	int number_of_previous_points = previous_x_vals.size();
	// Variable to indicate whether lane shifting is needed or not
	bool tooClose = false;

	/*** Use Sensor Fusion ***/
	// Step 1: Check sensor fusion data for cars in our lane
	// Set the target s to the expected s at the end of the previously planned path
	if (number_of_previous_points > 0)
		car_s = end_path_s;
	for (int i = 0; i < sensor_fusion.size(); i++)
	{
		float d = sensor_fusion[i][6];
		// If this car is within ego lane
		if ((d < (LANE_WIDTH/2) + LANE_WIDTH * lane + (LANE_WIDTH/2)) && 
			(d >(LANE_WIDTH / 2) + LANE_WIDTH * lane - (LANE_WIDTH / 2)))
		{
			double vx = sensor_fusion[i][3];
			double vy = sensor_fusion[i][4];
			double check_speed = sqrt(vx*vx + vy*vy);
			double check_car_s = sensor_fusion[i][5];

			// Check where this car will be at the end of the already planned path
			check_car_s += (double)number_of_previous_points * TRAJECTORY_DELTA * check_speed;

			// If it will be only within small distance from us: think about shifting lanes
			if ((check_car_s > car_s) && (check_car_s - car_s < MINIMUM_SEPARATION))
				tooClose = true;
		}
	}
	// Check whether the car is in lane center or not.
	// If it is not in lane center: it is shifting lanes
	bool bIsShiftingLanes = true;
	for (int i = 0; i < NUMBER_OF_LANES; i++)
	{
		if (abs(car_d - ((LANE_WIDTH / 2) + LANE_WIDTH * i)) < 1)
		{
			bIsShiftingLanes = false;
			break;
		}
	}
	/*** Try lane shifts if a car will be too close and if not already shifting lanes ***/
	if (tooClose && !bIsShiftingLanes)
	{
		// First, to the left
		bool bCanGoLeft = true;
		// If it is not the left most lane
		if (lane > 0)
		{
			for (int i = 0; i < sensor_fusion.size(); i++)
			{
				float d = sensor_fusion[i][6];
				// If a car is within this lane
				if ((d < (LANE_WIDTH / 2) + LANE_WIDTH * (lane - 1) + (LANE_WIDTH / 2)) &&
					(d > (LANE_WIDTH / 2) + LANE_WIDTH * (lane - 1) - (LANE_WIDTH / 2)))
				{
					double vx = sensor_fusion[i][3];
					double vy = sensor_fusion[i][4];
					double check_speed = sqrt(vx*vx + vy*vy);
					double check_car_s = sensor_fusion[i][5];

					// Check where this car will be at the end of the already planned path
					check_car_s += (double)number_of_previous_points * TRAJECTORY_DELTA * check_speed;

					// IF this car is too near to our back or not too far upfront, there is no added
					// value from lane shifting
					if ( ((check_car_s > car_s) && (check_car_s - car_s < MINIMUM_SEPARATION_FRONT)) ||
						 ((car_s > check_car_s) && (car_s - check_car_s < MINIMUM_SEPARATION_BACK)) )
					{
						bCanGoLeft = false;
						break;
					}
				}
			}
		}
		else
			bCanGoLeft = false;
		// IF shifting is OK
		if (bCanGoLeft)
		{
			// Go left without decreasing speed
			lane--;
			tooClose = false;
		}
		else
		{
			// Then, to the right
			bool bCanGoRight = true;
			// If it is not the right most lane
			if (lane < NUMBER_OF_LANES - 1)
			{
				for (int i = 0; i < sensor_fusion.size(); i++)
				{
					float d = sensor_fusion[i][6];
					// If a car is within this lane
					if ((d < (LANE_WIDTH / 2) + LANE_WIDTH * (lane + 1) + (LANE_WIDTH / 2)) &&
						(d > (LANE_WIDTH / 2) + LANE_WIDTH * (lane + 1) - (LANE_WIDTH / 2)))
					{
						double vx = sensor_fusion[i][3];
						double vy = sensor_fusion[i][4];
						double check_speed = sqrt(vx*vx + vy*vy);
						double check_car_s = sensor_fusion[i][5];

						// Check where this car will be at the end of the already planned path
						check_car_s += (double)number_of_previous_points * TRAJECTORY_DELTA * check_speed;

						// IF this car is too near to our back or not too far upfront, there is no added
						// value from lane shifting
						if (((check_car_s > car_s) && (check_car_s - car_s < MINIMUM_SEPARATION_FRONT)) ||
							((car_s > check_car_s) && (car_s - check_car_s < MINIMUM_SEPARATION_BACK)))
						{
							bCanGoRight = false;
							break;
						}
					}
				}
			}
			else
				bCanGoRight = false;
			// IF shifting is OK
			if (bCanGoRight)
			{
				// Go right without decreasing speed
				lane++;
				tooClose = false;
			}
		}
	}
	/*** Avoiding Jerk ***/
	// IF the car is too close: decelerate
	if (tooClose)
		ref_vel -= MAXIMUM_ACCELERATION; // 5m/s2
	// ELSE IF target speed is not reached: accelerate
	else if (ref_vel < TARGET_SPEED)
		ref_vel += MAXIMUM_ACCELERATION; // 5m/s2

	/*** Drive Straight in Lane Smoothly ***/
	// Step 1: take last 2 points of previous path as reference points
	if (number_of_previous_points < 2)
	{
		double prev_car_x = car_x - cos(car_yaw);
		double prev_car_y = car_y - sin(car_yaw);

		ptsx.push_back(prev_car_x);
		ptsx.push_back(car_x);
		ptsy.push_back(prev_car_y);
		ptsy.push_back(car_y);
	}
	else
	{
		ref_x = previous_x_vals[number_of_previous_points - 1];
		ref_y = previous_y_vals[number_of_previous_points - 1];
		double prev_ref_x = previous_x_vals[number_of_previous_points - 2];
		double prev_ref_y = previous_y_vals[number_of_previous_points - 2];
		ref_yaw = atan2(ref_y - prev_ref_y, ref_x - prev_ref_x);
		ptsx.push_back(prev_ref_x);
		ptsx.push_back(ref_x);
		ptsy.push_back(prev_ref_y);
		ptsy.push_back(ref_y);
	}

	// Step 2: Get Fernet points for 3 points with 30 meters distance (s+30, s+60, s+90)
	// Also, go to lane center
	vector<double> next_wp_0 = getXY(car_s + MINIMUM_SEPARATION, (LANE_WIDTH / 2) + LANE_WIDTH * lane);
	vector<double> next_wp_1 = getXY(car_s + 2*MINIMUM_SEPARATION, (LANE_WIDTH / 2) + LANE_WIDTH * lane);
	vector<double> next_wp_2 = getXY(car_s + 3* MINIMUM_SEPARATION, (LANE_WIDTH / 2) + LANE_WIDTH * lane);
	ptsx.push_back(next_wp_0[0]);
	ptsx.push_back(next_wp_1[0]);
	ptsx.push_back(next_wp_2[0]);
	ptsy.push_back(next_wp_0[1]);
	ptsy.push_back(next_wp_1[1]);
	ptsy.push_back(next_wp_2[1]);

	// Step 3: Shift the points to be in the car's coordinate system
	for (int i = 0; i < ptsx.size(); i++)
	{
		double shift_x = ptsx[i] - ref_x;
		double shift_y = ptsy[i] - ref_y;
		ptsx[i] = shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw);
		ptsy[i] = shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw);
	}

	// Step 4: Use previous points
	for (int i = 0; i < number_of_previous_points; i++)
	{
		next_x_vals.push_back(previous_x_vals[i]);
		next_y_vals.push_back(previous_y_vals[i]);
	}

	// Step 5: use spline to determine new points
	tk::spline s;
	s.set_points(ptsx, ptsy);
	double target_x = MINIMUM_SEPARATION;
	double target_y = s(target_x);
	double target_dist = sqrt(target_x * target_x + target_y * target_y);
	double x_add_on = 0;

	for (int i = 1; i <= NUMBER_OF_POINTS_PER_TRAJECTORY - number_of_previous_points; i++)
	{
		// 2.24 to convert mph to mps
		double N = (target_dist / (TRAJECTORY_DELTA * ref_vel / 2.24));
		double x_point = x_add_on + target_x / N;
		double y_point = s(x_point);
		x_add_on = x_point;
		double x_ref = x_point;
		double y_ref = y_point;
		x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
		y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);
		x_point += ref_x;
		y_point += ref_y;

		next_x_vals.push_back(x_point);
		next_y_vals.push_back(y_point);
	}
	return;
}