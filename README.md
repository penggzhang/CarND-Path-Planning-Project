# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
## Simulator
Term3 Simulator which contains the Path Planning Project can be downloaded from the [releases tab] (https://github.com/udacity/self-driving-car-sim/releases).

## Goals
In this project, the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The car's localization and sensor fusion data will be provided, and there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

## Data

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

Below is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using [spline] (http://kluge.in-chemnitz.de/opensource/spline/). Spline function is in a single hearder file and really easy to use.

---

## Model Implementation

The whole path planning work can be divided into two steps: Behavior Planning and Trajectory Generation. First, based on its own localization and predictions for other vehicles, ego car will decide to take which behavior, e.g. Keep Lane, Lane Change Right, or Lane Change Left. Then, with the behavior as input, a smooth or say jerk minimizing trajectory will be generated. Given such trajectory, ego car will visit sequentially and thus get its path.
 
### 1. Behavior Planning

***Objective*** Avoid crashing into the slower moving car ahead and try to change lanes.

***How to*** If there is a car ahead going slow, ego car will lower reference velocity to avoid crashing into it. Then check if it is feasible to change lanes, that is if the adjacent lane is clear of other traffic. If feasible, flag lane change and shift lanes.

***Code*** `main.cpp` line `329 ~ 391`.

* First, check if there is any other car in the same lane as ego car. And check if it is ahead of and too close to ego car. If positive, flag `too_close` will be marked.
* Then, ego car has to decide which behavior should be taken, based on cost function.
  * Possible behaviors include: Keep Lane, Lane Change Right, or Lane Change Left, which are named as 'state'.
  * `calculate_cost` function is called, and a scalar cost will be calculated for each state as the evaluation criterion.
  * Select the minimum cost behavior for ego car to take.
* Moreover, decide how to decelerate if the slower moving car ahead is too close, or accelerate if no car ahead is too close and current reference velocity is less than target speed.
  * The magnitude of reference velocity change is calculated as: `0.2 m/s^2 * 1 s * 3600 s / 1609 m = 0.448`

***Code*** `main.cpp` line `170 ~ 229`.

* Cost function `calculate_cost`: calculate cost for reaching a target state. This function takes a target state, sensor fusion data of other cars, ego car's lane, s and speed values, and then returns cost.
  * A simple but effective logic is applied here. If there is a gap available in the target adjacent lane, the cost for this target state will be reduced. In another word, this is like a reward that leads ego car to make the lane shift decision.
  * If a gap in adjacent lane cannot be found, then the cost for the target state will be increased. This is telling ego car that such a state shall not be taken.
  * `check_gap` function: check if there is a gap available in the adjacent lane. If there is a car in the range of [current s ~ future s] of ego car, or if the car is within a range of 10 meter around ego car, then the gap is not available.

### 2. Trajectory Generation

***Objective*** Create a list of waypoints that ego car visits sequentially every 0.02 seconds.

***How to*** Create a list of spaced waypoints, including a starting reference and several forward waypoints. Use spline to fit these waypoints, and then interpolate them with more points that control speed.

***Code*** `main.cpp` line `401 ~ 507`.

* First, find the starting reference. Either reference the point where ego car is, or the end point of previous path.
* Then, in Frenet, along s dimension, add three evenly 30m spaced points ahead of the starting reference. And d values are correspondingly determined by the lane where ego car is.
* Transform the five points' global xy coordinates to local coodinates at the reference point and yaw angle.
* Spline the points in local coordinates.
* Break up the segment between the five spline points so that ego car will go at reference speed. The interpolated breakup points will be transformed back to global coordinates and then filled in the rest of splined curve.
* Previous path points will also be used for better transition between planning cycles. 
* Finally, there will be 50 waypoints in total generated for path planner.

## Test Run Video

A [video](https://youtu.be/hyLmDJhNYFg) shows ego car is able to drive at least 4.32 miles without incident.

---

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```