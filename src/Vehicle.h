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
    
  double lane;
  
  double ego_vel;   // start with 0

  double target_speed;  // 49.5 mph

  string state;  // "KL" , "LCL", "LCR"
  
  bool too_close; //flag sets when ego vehicle is nearer to front vehicle in same lane
  double vehicle_gap_factor;

  /**
  * Constructor
  */
  Vehicle();
  
  /**
  * Destructor
  */
  virtual ~Vehicle();

  void configure(vector<int> road_data);

  vector<double> behavior_planning(vector<vector<double>> sensor_fusion, vector<double> previous_path_x, double car_s, double end_path_s);

private:
  void update_state(vector<vector<double>> sensor_fusion, vector<double> previous_path_x,double car_s, double end_path_s); // sensor_fusion data
  
  void realize_state();

  void realize_keep_lane();

  void realize_lane_change_left();

  void realize_lane_change_right();
    
  double cost_keep_lane(vector<vector<double>> sensor_fusion, vector<double> previous_path_x, double car_s, double end_path_s);

  double cost_lane_change(vector<vector<double>> sensor_fusion, vector<double> previous_path_x, double car_s, double end_path_s, string direction);
  
  string mincost_state(double cost_KL, double cost_LCL, double cost_LCR, vector<string> possible_states);  
};

#endif
