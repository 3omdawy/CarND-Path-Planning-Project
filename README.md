# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
  * Used version: 3.8.2
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
  * Used version: 0.6.2 Beta
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
  * Windows: Refer to the section "Windows Install Instructions" [here](https://github.com/swirlingsand/CarND-PID-Control-Project)
   
### Simulator. You can download the Term3 Simulator BETA which contains the Path Planning Project from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).

In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit.
Car's localization and sensor fusion data will be provided, there is also a sparse map list of waypoints around the highway.
The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too.
The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another.
The car should be able to make one complete loop around the 6946m highway.
Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.
5. Open the simulator and check the results

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: Currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points used to have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. We would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

## Source Files

1. spline.h: used to make a smooth path by spline function. It is taken as it is from http://kluge.in-chemnitz.de/opensource/spline/,
2. json.hpp: for parsing messgaes with simulator
3. main.cpp: parses messages from/to simulator and calls path planner to get the path
4. Helpers.h and Helpers.cpp: some helper functions (mainly to transform cartesian coordinates to Fernet and vice versa)
5. Configurations.h: configuration parameters of the project (lane width, maximum speed ...)
6. PathPlanner.h and PathPlanner.cpp: the logic of the path planner

## Path Generation Algorithm

1. Use sensor fusion to check the cars in ego lane
2. IF a car is too close: try lane shifting (left then right)
3. Shifting will only be decided if there is enough room behind and in front of the car in the new lane. Note: shifting is an atomic decision (i.e. after deciding to shift to a new lane, we will not think about shifting again till we arive to the new lane. This is done to avoid hanging between lanes)
4. Decide whether to accelerate or decelerate according to the current speed and the nearness of cars
5. Generate the next path given the previous path and new points (predicted) using spline function for smoothness

## Rubric items

Items in [rubric](https://review.udacity.com/#!/rubrics/1020/view) are met:

1. Code compilation: I compiled the code using VS2017 but I did not change anything related to make or cmake so it should compile normally

2. "The car is able to drive at least 4.32 miles without incident.": refer to `img` folder, car was able to drive for > 20 mins (then I stopped the simulator)

3. "The car drives according to the speed limit": yes. Car tries to accelerate to reach the speed limit of 50mph as much as possible:
```
	/*** Avoiding Jerk ***/
	// IF the car is too close: decelerate
	if (tooClose)
		ref_vel -= MAXIMUM_ACCELERATION; // 5m/s2
	// ELSE IF target speed is not reached: accelerate
	else if (ref_vel < TARGET_SPEED)
		ref_vel += MAXIMUM_ACCELERATION; // 5m/s2
```
Note: 
```
// Target speed (in mph)
#define TARGET_SPEED 49.5
```

4. "Max Acceleration and Jerk are not Exceeded.": yes, the car always accelerates or decelerates slowly
```
// Maximum acceleration (in m/s2)
#define MAXIMUM_ACCELERATION 0.224
```

5. "Car does not have collisions.": yes ... refer to the logic for determining `tooClose` whether a car is close or not. If a car is close within a good distance, deceleration is done
```
// Minimum separation between us and next car in the lane (in m)
#define MINIMUM_SEPARATION 30
```

6. "The car stays in its lane, except for the time between changing lanes.": yes ... for this part, lane changing is done atomically (i.e. whenever we decide to shift lanes, we do not think about lane shifting again except if we went to the new lane to avoid being in the middle between 2 lanes for a long time)
```
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
```

7. "The car is able to change lanes": yes, refer to the logic done under the part `if (tooClose && !bIsShiftingLanes)`. Shifting is done when a front car is too close and in an adjacent lane, there is room in front and behind of our car
```
// Minimum separation between us and previous car in the lane (in m)
#define MINIMUM_SEPARATION_BACK 20
// Minimum separation between us and next car in a lane for shifting to be beneficial (in m)
#define MINIMUM_SEPARATION_FRONT 45
```

8. "There is a reflection on how to generate paths.": path is generated according to the ReadMe video using the following steps:
	a. Step 1: take last 2 points of previous path as reference points
	b. Step 2: Get Fernet points for 3 points with 30 meters distance (s+30, s+60, s+90). Also, go to lane center.
	c. Step 3: Shift the points to be in the car's coordinate system
	d. Step 4: Use all remaining points from previous path
	e. Step 5: Use spline to determine new points
	f. Step 6: Determine spacing between points (as shown in PointsSpacing.PNG)
	g. Step 7: Shift the points back to global coordinates
	h. Step 8: Add them to next path points