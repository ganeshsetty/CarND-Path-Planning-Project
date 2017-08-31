#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iterator>

using namespace std;

class Vehicle {
public:

  
  int preferred_buffer = 30; // 30 meters impacts "keep lane" behavior.

  double lane;

  //int s;

  double ego_vel;   // start with 0

  double target_speed;  // 49.5 mph

  int lanes_available;  

  string state;  // "KL" , "LCL", "LCR"
  
  bool too_close;

  /**
  * Constructor
  */
  Vehicle();
  //Vehicle(int lane, int s,int d, int v);

  /**
  * Destructor
  */
  virtual ~Vehicle();

  void configure(vector<int> road_data);

  vector<double> behavior_planning(vector<vector<double>> sensor_fusion, vector<double> previous_path_x, double car_s, double end_path_s);

  //void path_planning();

private:

  void update_state(vector<vector<double>> sensor_fusion, vector<double> previous_path_x,double car_s, double end_path_s); // sensor_fusion data

  //void generate_trajectory();

  void realize_state();

  void realize_keep_lane();

  void realize_lane_change_left();

  void realize_lane_change_right();
  //void realize_prep_lane_change(map<int,vector< vector<int> > > predictions, string direction);
  
  double cost_keep_lane(vector<vector<double>> sensor_fusion, vector<double> previous_path_x, double car_s, double end_path_s);

  double cost_lane_change_left(vector<vector<double>> sensor_fusion, vector<double> previous_path_x, double car_s, double end_path_s);

  double cost_lane_change_right(vector<vector<double>> sensor_fusion, vector<double> previous_path_x, double car_s, double end_path_s);
  
  string mincost(double cost_KL, double cost_LCL, double cost_LCR, vector<string> possible_states);  
};

#endif
