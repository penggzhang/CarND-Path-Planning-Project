#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include <string>

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Compute Euclidean distance between two points
double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Find the closest waypoint for the given coordinates
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{
	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x, y, map_x, map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}
	}

	return closestWaypoint;
}

// Find next waypoint for the given coordinates and orientation
int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
	int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

    // Calculate the turning angle from current orientation toward the closest waypoint
	double heading = atan2((map_y - y), (map_x - x));
	double angle = abs(theta-heading);

    // If the turning angle is beyond feasible range,
    // then try the waypoint after next one.
    if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
	int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);

	int prev_wp;
	prev_wp = next_wp - 1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size() - 1;
	}
    
    // Offset between two neighboring waypoints' coordinates
    // gives the x,y components of the connecting vector n.
    double n_x = maps_x[next_wp] - maps_x[prev_wp];
	double n_y = maps_y[next_wp] - maps_y[prev_wp];
    // Offset between the given point and previous waypoint
    // gives the x,y components of another connecting vector x.
    double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
	double proj_x = proj_norm * n_x;
	double proj_y = proj_norm * n_y;

    // find frenet d
	double frenet_d = distance(x_x, x_y, proj_x, proj_y);

	// see if d value is positive or negative by comparing it to a center point
	double center_x = 1000 - maps_x[prev_wp];
	double center_y = 2000 - maps_y[prev_wp];
	double centerToPos = distance(center_x, center_y, x_x, x_y);
	double centerToRef = distance(center_x, center_y, proj_x, proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i], maps_y[i], maps_x[i+1], maps_y[i+1]);
	}

	frenet_s += distance(0, 0, proj_x, proj_y);

	return {frenet_s, frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
	// Find the two waypoints where the given point locates between
    int prev_wp = -1;
	while(s > maps_s[prev_wp + 1] && (prev_wp < (int)(maps_s.size() - 1) ))
	{
		prev_wp++;
	}
	int wp2 = (prev_wp + 1) % maps_x.size();
    
    // The heading between the two waypoints
    double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
	
    // Find the x,y,s of the projecton point along the segment
	double seg_s = (s - maps_s[prev_wp]);
	double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
	double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

    // Perpendicular heading from the projection point to the given point
	double perp_heading = heading - pi()/2;

    // Calculate corresponding x,y with the perpendicular heading and given d
	double x = seg_x + d * cos(perp_heading);
	double y = seg_y + d * sin(perp_heading);

	return {x, y};
}

// Check if there is a gap available
bool check_gap(int lane, vector<vector<double> >sensor_fusion, double ego_s, double ego_speed, int path_size) {
    // Initial set
    bool gap_available = true;
    
    // Project ego car's s forward in time
    double ego_s_next = ego_s + ego_speed * (double)path_size * 0.02;
    
    // Iterate over all other cars
    for (int i = 0; i < sensor_fusion.size(); i++) {
        // Check if any other car is in this lane
        if ((sensor_fusion[i][6] > (4*lane)) && (sensor_fusion[i][6] < (4*lane + 4))) {
            // Find the car's s, speed and future s
            double check_car_s = sensor_fusion[i][5];
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_car_v = sqrt(vx * vx + vy * vy);
            double check_car_s_next = check_car_s + (double)path_size * 0.02 * check_car_v;
            
            // If the car will be in the range of [current s ~ future s] of ego car, or if the car is within a range of 10 meter around ego car, then flag the gap as unavailable.
            if ((check_car_s_next >= ego_s && check_car_s_next <= ego_s_next) ||
                (abs(check_car_s - ego_s) < 10.0)) {
                gap_available = false;
                return gap_available;
            }
        }
    }
    
    return gap_available;
}

// Calculate cost for reaching a target state
double calculate_cost(string state, int lane, vector<vector<double> > sensor_fusion, double car_s, double car_speed, int path_size) {
    // Takes a target state, sensor fusion data of other cars, ego car's lane, s and speed values. Returns cost.
    
    double cost = 0.0;
    // If target state is "Keep Lane", then return 0.
    if (state.compare("KL") == 0) {
        return cost;
    }
    else {
        // Find the target lane to go
        int target_lane;
        if (state.compare("LCL") == 0) {
            target_lane = lane - 1;
        } else if (state.compare("LCR") == 0) {
            target_lane = lane + 1;
        }
        
        // Check if there is a gap available, in the target lane, for lane shift.
        if (check_gap(target_lane, sensor_fusion, car_s, car_speed, path_size) == true) {
            // If positive, reduce the cost by 1.
            cost -= 1.0;
        } else {
            // Otherwise, increment the cost by 1.
            cost += 1.0;
        }
        
        return cost;
    }
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
  	float s;
  	float d_x;
  	float d_y;
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
    
  // Start in lane 1
  int lane = 1;
    
  // Target velocity mph
  double ref_vel = 0.0;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&lane,&ref_vel](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Ego car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor fusion data, a list of all other cars on the same side of the road.
          	vector<vector<double> > sensor_fusion = j[1]["sensor_fusion"];
            
            // Size of path
            int path_size = previous_path_x.size();
            
            // Use previous path for better transition of planning
            if (path_size > 0) {
                car_s = end_path_s;
            }
            
            // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
            
            /* 
             Behavior Planning
             - Objective: Avoid crashing into the car ahead and try to change lanes.
             - Logic: If there is a car ahead going slow, the ego car will lower reference velocity to avoid crashing into it. Then check if it is feasible to change lanes. If feasible, flag lane change and shift lanes.
             */

            // First, check if there is any other car in the same lane as ego car. And check if it is ahead of and too close to ego car.
            bool too_close = false;
            // Iterate over all other cars:
            for (int i = 0; i < sensor_fusion.size(); i++) {
                // Check if the car is in the ego lane
                float d = sensor_fusion[i][6];
                if (d > (4 * lane) && d < (4 * lane + 4)) {
                    // Find the car's speed
                    double vx = sensor_fusion[i][3];
                    double vy = sensor_fusion[i][4];
                    double check_car_v = sqrt(vx*vx + vy*vy);
                    
                    // Find the car's s
                    double check_car_s = sensor_fusion[i][5];
                    
                    // Project the s value forward in time
                    check_car_s += (double)path_size * 0.02 * check_car_v;
                    
                    // Check if the car will be in front of ego and within a certain gap, that means if it will be too close to ego.
                    if ((check_car_s > car_s) && ((check_car_s - car_s) < 30)) {
                        // Flag it
                        too_close = true;
                        
                        // Try to change lanes.
                        
                        // List possible states for behavior planning.
                        vector<string> states = {"KL", "LCL", "LCR"};
                        // Remove infeasible states.
                        if (lane == 0) {
                            states.erase(states.begin() + 1);
                        }
                        if (lane == 2) {
                            states.erase(states.begin()+2);
                        }

                        // Calculate cost for each state
                        vector<double> costs;
                        for (int i = 0 ; i < states.size(); i++) {
                            double cost = calculate_cost(states[i], lane, sensor_fusion, car_s, car_speed, path_size);
                            costs.push_back(cost);
                        }
                        
                        // Find the minimum cost state that ego car shall pursue
                        vector<double>::iterator min_ind_ptr = min_element(costs.begin(), costs.end());
                        long min_ind = distance(costs.begin(), min_ind_ptr);
                        
                        // Flag corresponding lane change
                        if (states[min_ind].compare("LCL") == 0) {
                            lane -= 1;
                        } else if (states[min_ind].compare("LCR") == 0) {
                            lane += 1;
                        }                        
                    }
                }
            }
            // Decide how to decelerate if the car ahead is too close, or accelerate if no car ahead is too close and current reference velocity is less than target speed.
            // The magnitude of reference velocity change is calculated as:
            // 0.2 m/s^2 * 1 s * 3600 s / 1609 m = 0.448
            if (too_close) {
                ref_vel -= 0.448;
            }
            else if (ref_vel < 49.5) {
                ref_vel += 0.448;
            }
            
            
            /*
             Trajectory Generation
             - Create a list of spaced waypoints, including a starting reference and several forward waypoints.
             - Use spline to fit these waypoints, and then interpolate them with more points that control speed.
             */
            
            // Define a list points
            vector<double> ptsx;
            vector<double> ptsy;
            
            // Find the starting reference.
            // Either reference the point where ego car is, or the end point of previous path.
            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);
            
            // If the previous path is almost empty, use the point where ego car is as starting reference.
            if (path_size < 2) {
                // Find two points that make the path tangent to ego car
                double prev_car_x = car_x - cos(car_yaw);
                double prev_car_y = car_y - sin(car_yaw);
                
                ptsx.push_back(prev_car_x);
                ptsx.push_back(car_x);
                
                ptsy.push_back(prev_car_y);
                ptsy.push_back(car_y);
            }
            // Use the end point of previous path as starting reference
            else {
                // Redefine reference point
                ref_x = previous_path_x[path_size - 1];
                ref_y = previous_path_y[path_size - 1];
                
                // Find two points that make the path tangent to the end point of previous path
                double ref_x_prev = previous_path_x[path_size - 2];
                double ref_y_prev = previous_path_y[path_size - 2];
                ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
                
                ptsx.push_back(ref_x_prev);
                ptsx.push_back(ref_x);
                
                ptsy.push_back(ref_y_prev);
                ptsy.push_back(ref_y);
            }
            
            // In Frenet, along s dimension, add evenly 30m spaced points ahead of the starting reference.
            // And d values are correspondingly determined by the lane.
            vector<double> next_pt0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_pt1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_pt2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            
            ptsx.push_back(next_pt0[0]);
            ptsx.push_back(next_pt1[0]);
            ptsx.push_back(next_pt2[0]);
            
            ptsy.push_back(next_pt0[1]);
            ptsy.push_back(next_pt1[1]);
            ptsy.push_back(next_pt2[1]);
            
            // Transform all found points' global xy coordinates to local coodinates at the reference point and yaw angle
            for (int i = 0; i < ptsx.size(); i++) {
                double delta_x = ptsx[i] - ref_x;
                double delta_y = ptsy[i] - ref_y;
                
                ptsx[i] = (delta_x * cos(ref_yaw) + delta_y * sin(ref_yaw));
                ptsy[i] = (-delta_x * sin(ref_yaw) + delta_y * cos(ref_yaw));
            }
            
            // Spline the points in local coordinates
            tk::spline s;
            s.set_points(ptsx, ptsy);
            
            // Break up the segment between spline points so that the car will go at reference speed
            double segment_x = 30.0;
            double segment_y = s(segment_x);
            // Distance between two end points of the segment
            double segment_dist = sqrt(segment_x * segment_x + segment_y * segment_y);
            // The segment distance and reference velocity give the number of breakups, N.
            // Transform MPH velocity to meter per second, 1609 / 3600 = 0.4469.
            double N = segment_dist/(0.02 * ref_vel * 0.4469);
            
            // Define resulting points for planner
            vector<double> next_x_vals;
            vector<double> next_y_vals;
            
            // Use previous path points for better transition
            for (int i = 0; i < path_size; i++) {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
            }
 
            // Interpolate breakup points and fill them in the rest of splined curve.
            // Always output 50 points in total.
            double x_add_on = 0;
            for (int i = 0; i < 50 - path_size; i++) {
                
                // Interpolate breakup points in local coordinates
                double x_local = x_add_on + segment_x / N;
                double y_local = s(x_local);
                // Step up for interpolation
                x_add_on = x_local;
                
                // Transform to global coordinates by rotation and then translation
                double x_global = (x_local * cos(ref_yaw) - y_local * sin(ref_yaw));
                double y_global = (x_local * sin(ref_yaw) + y_local * cos(ref_yaw));
                
                x_global += ref_x;
                y_global += ref_y;
                
                // Add the resulting points
                next_x_vals.push_back(x_global);
                next_y_vals.push_back(y_global);
            }

            
            json msgJson;
            
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}