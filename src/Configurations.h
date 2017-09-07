#pragma once
// Simulation parameters
// Time between messages for trajectory planning (in s)
#define TRAJECTORY_DELTA 0.02

// Behavior parameters
// Target speed (in mph)
#define TARGET_SPEED 49.5
// Maximum acceleration (in m/s2)
#define MAXIMUM_ACCELERATION 0.224
// Minimum separation between us and next car in the lane (in m)
#define MINIMUM_SEPARATION 30
// Minimum separation between us and previous car in the lane (in m)
#define MINIMUM_SEPARATION_BACK 20
// Minimum separation between us and next car in a lane for shifting to be beneficial (in m)
#define MINIMUM_SEPARATION_FRONT 45

// Prediction parameters
// Number of lanes
#define NUMBER_OF_LANES 3
// Width of each lane (in meters)
#define LANE_WIDTH 4

// Trajectory parameters
// Number of points of next trajectory
#define NUMBER_OF_POINTS_PER_TRAJECTORY 50