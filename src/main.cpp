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

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}


vector<string> get_successor_states(string state, int lane){
  /*
  Provides the possible next states given the current state for the FSM
  discussed in the course, with the exception that lane changes happen
  instantaneously, so LCL and LCR can only transition back to KL.
  */
  int lanes_available = 3;
  vector<string> states;
  states.push_back("KL");
  if(state.compare("KL") == 0) {
      states.push_back("PLCL");
      states.push_back("PLCR");
  } else if (state.compare("PLCR") == 0) {
      if (lane < lanes_available - 1) { //if lane isn't far right lane
          states.push_back("PLCR");
          states.push_back("LCR");
      }
  } else if (state.compare("PLCL") == 0) {
      if (lane > 0) {
          states.push_back("PLCL");
          states.push_back("LCL");
      }
  }
  //If state is "LCL" or "LCR", then just return "KL"
  return states;
}


vector<double> get_avg_lane_speeds(vector<double> lane0, vector<double> lane1, vector<double> lane2){

  double lane0_speed = 0.0; double lane1_speed = 0.0; double lane2_speed = 0.0;
  vector<double> avg_lane_speeds = {0.0, 0.0, 0.0};

  //TEST LANE 0
  for(int i=0; i<lane0.size(); i++){lane0_speed += lane0[i];}
  avg_lane_speeds[0] = lane0_speed/lane0.size();

  //TEST LANE 1
  for(int i=0; i<lane1.size(); i++){lane1_speed += lane1[i];}
  avg_lane_speeds[1] = lane1_speed/lane1.size();

  //TEST LANE 2
  for(int i=0; i<lane2.size(); i++){lane2_speed += lane2[i];}
  avg_lane_speeds[2] = lane2_speed/lane2.size();


  return avg_lane_speeds;
}

vector<double> get_lane_costs(int current_lane, vector<double>lane_speeds, vector<string> safe_states, vector<int> num_obj_in_lane){

  int intended_lane;
  vector<double> lane_cost = {0.0, 0.0, 0.0};
  vector<string> prioritized_states;

  //PENALIZE MOVING INTO CROWDED LANES
  for(int j=0; j<num_obj_in_lane.size(); j++){
    lane_cost[j] += 10*num_obj_in_lane[j];
  }

  //PENALIZE SLOWER MOVING LANES
  for(int j=0; j<lane_speeds.size(); j++){
    cout << "Lane Speed " << j <<" = "<< lane_speeds[j] << endl;
    lane_cost[j] += (1/(lane_speeds[j]));
  }

  for(int i=0; i<safe_states.size(); i++){

    if(safe_states[i].compare("PLCL") == 0 || safe_states[i].compare("LCL") == 0){intended_lane = current_lane-1;}
    else if(safe_states[i].compare("PLCR") == 0 || safe_states[i].compare("LCR") == 0){intended_lane = current_lane+1;}
    else{intended_lane = current_lane;}

    //PENALIZE STAGNATION IF OTHER MANUEVERS AVAILABLE
    if(safe_states[i].compare("PLCL") == 0 || safe_states[i].compare("PLCR") == 0 || safe_states[i].compare("KL") == 0){
      lane_cost[current_lane] += 1;}

  }

  return lane_cost;
}

string get_best_state(int lane, vector<double> lane_costs, vector<string>safe_states){

  //DEBUG
  cout << "Lane 0 cost: " << lane_costs[0] << "   Lane 1 cost: " << lane_costs[1] << "    Lane 2 cost: " << lane_costs[2] << endl;
  //END DEBUG

  string best_state;
  int lowest_cost_lane;
  int intended_lane;

  double lowest_cost = 99;
  double temp_cost = 0;

  for(int i=0; i<lane_costs.size(); i++){
    temp_cost = lane_costs[i];
    if(temp_cost < lowest_cost){
      lowest_cost = temp_cost;
      lowest_cost_lane = i;
    }
  }

  if(lowest_cost_lane == lane){return "KL";}

  cout << "Lowest Cost Lane: " << lowest_cost_lane << endl;

  for(int i=0; i<safe_states.size(); i++){
    if(lane == 0){
      if(lowest_cost_lane == 2){
        if(lane_costs[1] < lane_costs[lane]){lowest_cost_lane = 1;}
      }
      if(lowest_cost_lane == 1 && safe_states[i].compare("LCR") == 0){return "LCR";}
    }
    if(lane == 2){
      if(lowest_cost_lane == 0){
        if(lane_costs[1] < lane_costs[lane]){lowest_cost_lane = 1;}
      }
      if(lowest_cost_lane == 1 && safe_states[i].compare("LCL") == 0){return "LCL";}

    }
    if(lane == 1){
      if(lowest_cost_lane == 0 && safe_states[i].compare("LCL") == 0){return "LCL";}
      else if(lowest_cost_lane == 2 && safe_states[i].compare("LCR") == 0){return "LCR";}
    }

  }

  for(int i=0; i<safe_states.size(); i++){
    if(lane == 0){
      if(lowest_cost_lane == 2){
        if(lane_costs[1] < lane_costs[lane]){lowest_cost_lane = 1;}
      }
      if(lowest_cost_lane == 1 && safe_states[i].compare("PLCR") == 0){return "PLCR";}
    }
    if(lane == 2){
      if(lowest_cost_lane == 0){
        if(lane_costs[1] < lane_costs[lane]){lowest_cost_lane = 1;}
      }
      if(lowest_cost_lane == 1 && safe_states[i].compare("PLCL") == 0){return "PLCL";}
    }
    if(lane == 1){
      if(lowest_cost_lane == 0 && safe_states[i].compare("PLCL") == 0){return "PLCL";}
      else if(lowest_cost_lane == 2 && safe_states[i].compare("PLCR") == 0){return "PLCR";}
    }
  }
  return "KL";//IF NO LANE IS FOUND, RETURN KL
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

  int lane = 1; //current lane tracker
  double ref_vel = 0.0; //mph
  string ego_state = "KL";
  vector<double> close_objects_lane0, close_objects_lane1, close_objects_lane2;
  vector<double> lane_speed_0, lane_speed_1, lane_speed_2;



  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &lane, &ref_vel, &ego_state, &close_objects_lane0, &close_objects_lane1, &close_objects_lane2, &lane_speed_0, &lane_speed_1, &lane_speed_2](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
          
        	// Main car's localization Data
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

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;
          	vector<double> avg_lane_speeds;

          	int prev_size = previous_path_x.size();

          	//SENSOR FUSION

          	if(prev_size > 0){
          	  car_s = end_path_s;
          	}

          	bool too_close = false;
          	bool left_lane_free = false;
          	bool right_lane_free = false;
          	//CREATE FUSED OBJECT LIST

            double forward_obj_vel;

          	//find ref vel
          	for(int i=0; i<sensor_fusion.size(); i++){
          	  //car is in my lane
          	  float d = sensor_fusion[i][6];
          	  int obj_id = sensor_fusion[i][0];
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx+vy*vy);
              double check_car_s = sensor_fusion[i][5];
              int fused_obj_lane;



              check_car_s += ((double)prev_size*0.02*check_speed);

              if(d < (2+4*0+2) && d > (2+4*0-2)){fused_obj_lane = 0;}
              else if(d < (2+4*1+2) && d > (2+4*1-2)){fused_obj_lane = 1;}
              else if(d < (2+4*2+2) && d > (2+4*2-2)){fused_obj_lane = 2;}

              if((check_car_s-car_s) < 25.0 && (check_car_s-car_s) > -10.0){
                //add vehicle ID to object list for given lane
                if(fused_obj_lane == 0 && find(close_objects_lane0.begin(), close_objects_lane0.end(), obj_id) == close_objects_lane0.end()){
                  close_objects_lane0.push_back(obj_id);
//                  lane_speed_0.push_back(check_speed);
                }
                else if(fused_obj_lane == 1 && find(close_objects_lane1.begin(), close_objects_lane1.end(), obj_id) == close_objects_lane1.end()){
                  close_objects_lane1.push_back(obj_id);
//                  lane_speed_1.push_back(check_speed);
                }
                else if(fused_obj_lane == 2 && find(close_objects_lane2.begin(), close_objects_lane2.end(), obj_id) == close_objects_lane2.end()){
                  close_objects_lane2.push_back(obj_id);
//                  lane_speed_2.push_back(check_speed);
                }
//                avg_lane_speeds = get_avg_lane_speeds(lane_speed_0,lane_speed_1,lane_speed_2);
              }

              else{
                if(fused_obj_lane == 0){
                  close_objects_lane0.erase(remove(close_objects_lane0.begin(), close_objects_lane0.end(), obj_id), close_objects_lane0.end());
//                  lane_speed_0.erase(remove(lane_speed_0.begin(), lane_speed_0.end(), check_speed), lane_speed_0.end());
                }
                else if(fused_obj_lane == 1){
                  close_objects_lane1.erase(remove(close_objects_lane1.begin(), close_objects_lane1.end(), obj_id), close_objects_lane1.end());
//                  lane_speed_1.erase(remove(lane_speed_1.begin(), lane_speed_1.end(),check_speed), lane_speed_1.end());
                }
                else if(fused_obj_lane == 2){
                  close_objects_lane2.erase(remove(close_objects_lane2.begin(), close_objects_lane2.end(),obj_id), close_objects_lane2.end());
//                  lane_speed_2.erase(remove(lane_speed_2.begin(), lane_speed_2.end(),check_speed), lane_speed_2.end());
                }
//                avg_lane_speeds = get_avg_lane_speeds(lane_speed_0,lane_speed_1,lane_speed_2);
              }

              //LANE SPEED LISTS

              if((check_car_s-car_s) < 100.0 && (check_car_s-car_s) > -15.0){
                if(fused_obj_lane == 0){lane_speed_0.push_back(check_speed);}
                else if(fused_obj_lane == 1){lane_speed_1.push_back(check_speed);}
                else if(fused_obj_lane == 2){lane_speed_2.push_back(check_speed);}
                avg_lane_speeds = get_avg_lane_speeds(lane_speed_0,lane_speed_1,lane_speed_2);
              }

              else{
                if(fused_obj_lane == 0){
                  lane_speed_0.erase(remove(lane_speed_0.begin(), lane_speed_0.end(), check_speed), lane_speed_0.end());
                }
                else if(fused_obj_lane == 1){
                  lane_speed_1.erase(remove(lane_speed_1.begin(), lane_speed_1.end(),check_speed), lane_speed_1.end());
                }
                else if(fused_obj_lane == 2){
                  lane_speed_2.erase(remove(lane_speed_2.begin(), lane_speed_2.end(),check_speed), lane_speed_2.end());
                }
                avg_lane_speeds = get_avg_lane_speeds(lane_speed_0,lane_speed_1,lane_speed_2);
              }

              //cout << close_objects_lane0.size() << " " << close_objects_lane1.size() << " "<< close_objects_lane2.size() << endl;
              if(close_objects_lane0.size() == 0){lane_speed_0 = {50.0};}
              if(close_objects_lane1.size() == 0){lane_speed_1 = {50.0};}
              if(close_objects_lane2.size() == 0){lane_speed_2 = {50.0};}

              //END CREATE FUSED OBJECT LISTS
              //
              //DEBUG*********

              //cout << "ID: " << sensor_fusion[i][1] << "   Lane: " << fused_obj_lane << "   Distance (S): " << check_car_s-car_s << endl;

              //END DEBUG****

              //CALCULATE LANE SPEEDS
              //TODO

          	  if(d < (2+4*lane+2) && d > (2+4*lane-2))
          	  {
          	    if((check_car_s > car_s) && (check_car_s-car_s) < 30){ //within 30 meters
          	      //take action to prevent collision with vehicle
          	      too_close = true;
          	      forward_obj_vel = check_speed;
          	      //logic for lane changes should be here
          	      //finite state machine
          	      //cost function
          	      //etc

          	      vector<string> successor_states = get_successor_states(ego_state, lane);

          	      //DEBUG
          	      //cout << "Current Lane: " << lane << ", " << "Current State: " << ego_state << endl;
          	      //END DEBUG

          	      vector<string> safe_states;

          	      string best_state;


          	      if(lane == 0 && close_objects_lane1.size() == 0){right_lane_free = true;}
          	      else if(lane == 2 && close_objects_lane1.size() == 0){left_lane_free = true;}
          	      else if(lane == 1){
          	        if(close_objects_lane0.size() == 0){left_lane_free = true;}
          	        if(close_objects_lane2.size() == 0){right_lane_free = true;}
          	      }

          	      for(int i=0; i<successor_states.size(); i++){
          	        if(successor_states[i].compare("KL") == 0){
          	          safe_states.push_back(successor_states[i]);
          	        }
          	        if(successor_states[i].compare("PLCL") == 0){
          	          if(left_lane_free){
          	            safe_states.push_back(successor_states[i]);
          	          }
          	          else{cout << " ";}
          	        }
          	        if(successor_states[i].compare("PLCR") == 0){
          	          if(right_lane_free){

          	            safe_states.push_back(successor_states[i]);
          	          }
          	          else{cout << " ";}
          	        }
          	        if(successor_states[i].compare("LCL") == 0){
          	          if(left_lane_free){
          	            safe_states.push_back(successor_states[i]);
          	          }
          	          else{cout << "Left Lane Not Free ";}
          	        }
          	        if(successor_states[i].compare("LCR") == 0){
          	          if(right_lane_free){
          	            safe_states.push_back(successor_states[i]);
          	          }
          	          else{cout << "Right Lane Not Free ";}
          	        }

          	      }

          	      vector<int> num_obj_in_lanes;
          	      num_obj_in_lanes.push_back(close_objects_lane0.size());
          	      num_obj_in_lanes.push_back(close_objects_lane1.size());
          	      num_obj_in_lanes.push_back(close_objects_lane2.size());
          	      vector<double> lane_costs = get_lane_costs(lane, avg_lane_speeds, safe_states, num_obj_in_lanes);
          	      best_state = get_best_state(lane, lane_costs, safe_states);

//          	      for(int i=0; i<safe_states.size(); i++){
//          	        if(safe_states[i].compare("LCL") == 0){
//          	          best_state = safe_states[i];
//          	        }
//          	        else if(safe_states[i].compare("LCR") == 0){
//                      best_state = safe_states[i];
//                    }
//                    else if(safe_states[i].compare("PLCL") == 0){
//                      best_state = safe_states[i];
//                    }
//                    else if(safe_states[i].compare("PLCR") == 0){
//                      best_state = safe_states[i];
//                    }
//                    else if(safe_states[i].compare("KL") == 0){
//                      best_state = safe_states[i];
//                    }
//          	      }

          	      cout << best_state << endl;
          	      if(best_state == "PLCL"){
          	        //TODO: safety checks should be done here
          	        cout << "Preparing Lane Change Left" << endl;
          	      }
          	      else if(best_state == "LCL" && (d - (2+4*lane) < 0.1)){
          	        lane -= 1;
          	      }
          	      else if(best_state == "PLCR" ){
                    //TODO: safety checks should be done here
          	        cout << "Preparing Lane Change Right" << endl;

          	      }
          	      else if(best_state == "LCR" && (d - (2+4*lane) < 0.1)){
          	        lane += 1;
          	      }
          	      else if(best_state == "KL"){/*cout<< "Keep Lane" << endl;*/}


          	      //DEBUG
          	      //cout << "Best State: " << best_state << ", " << "Chosen Lane: " << lane << endl;
          	      //END DEBUG

          	      ego_state = best_state;
          	    }
          	  }
          	}

          	//prevent collision logic

          	if(too_close && ref_vel/2.24 > forward_obj_vel + 0.2){
          	  ref_vel -= 0.184;
          	}
          	else if(too_close && ref_vel/2.24 > forward_obj_vel){
          	  ref_vel -= 0.224;
          	}
          	else if(ref_vel < 49.5){
          	  ref_vel += 0.224;
          	}
          	//END SENSOR FUSION

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds

          	// Define vector of widely spaced reference waypoints
          	vector<double> ptsx, ptsy;

          	//define reference states
          	double ref_x = car_x;
          	double ref_y = car_y;
          	double ref_yaw = deg2rad(car_yaw);

          	//set first reference points based on previous vehicle state
          	if(prev_size < 2){
          	  double prev_car_x = car_x - cos(car_yaw);
          	  double prev_car_y = car_y - sin(car_yaw);

          	  ptsx.push_back(prev_car_x);
          	  ptsx.push_back(car_x);
              ptsy.push_back(prev_car_y);
              ptsy.push_back(car_y);
          	}
          	else{
          	  //redefine reference states as previous path end points
          	  ref_x = previous_path_x[prev_size-1];
          	  ref_y = previous_path_y[prev_size-1];

          	  double ref_x_prev = previous_path_x[prev_size-2];
          	  double ref_y_prev = previous_path_y[prev_size-2];
          	  ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

          	  //use the two points that make the path tangent to the previous path's end point
          	  ptsx.push_back(ref_x_prev);
          	  ptsx.push_back(ref_x);
          	  ptsy.push_back(ref_y_prev);
          	  ptsy.push_back(ref_y);
          	}

          	//Set spaced reference waypoints at 30, 60 and 90 meters
          	vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          	vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          	vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          	ptsx.push_back(next_wp0[0]);
          	ptsx.push_back(next_wp1[0]);
          	ptsx.push_back(next_wp2[0]);
          	ptsy.push_back(next_wp0[1]);
          	ptsy.push_back(next_wp1[1]);
          	ptsy.push_back(next_wp2[1]);

          	for(int i=0; i<ptsx.size(); i++){
          	  //shift car ref heading angle to 0 deg
          	  double shift_x = ptsx[i]-ref_x;
          	  double shift_y = ptsy[i]-ref_y;

          	  ptsx[i] = (shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
          	  ptsy[i] = (shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
          	}

          	//create a spline and set x,y points to the spline
          	tk::spline s;
          	s.set_points(ptsx, ptsy);

          	//start with all previous path points in planner points
          	for(int i=0; i<previous_path_x.size(); i++){
          	  next_x_vals.push_back(previous_path_x[i]);
          	  next_y_vals.push_back(previous_path_y[i]);
          	}

          	//calculate how to break up spline points for speed
          	double target_x = 30.; //horizon defined by distance (m)
          	double target_y = s(target_x);
          	double target_dist = sqrt((target_x)*(target_x) + (target_y)*(target_y));
          	double x_add_on = 0;

          	//fill remaining points in planned points
          	for(int i=1; i<=50-previous_path_x.size(); i++){
          	  double N = (target_dist/(0.02*ref_vel/2.24));
          	  double x_point = x_add_on+(target_x)/N;
          	  double y_point = s(x_point);

          	  x_add_on = x_point;

          	  double x_ref = x_point;
          	  double y_ref = y_point;

          	  //re-rotate reference frame back to normal heading angle
          	  x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
          	  y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));

          	  x_point += ref_x;
          	  y_point += ref_y;

          	  next_x_vals.push_back(x_point);
          	  next_y_vals.push_back(y_point);
          	}

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
