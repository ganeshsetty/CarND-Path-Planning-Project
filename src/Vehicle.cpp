#include <iostream>
#include "Vehicle.h"
#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <iterator>
#include <cassert>

#define STOP_COST 0.5
#define NO_VEHICLE_ID 999
/**
 * Initializes Vehicle
 */
Vehicle::Vehicle() {

    this->lane = 1;
    
    this->ego_vel = 0.0;
    
    this->target_speed = 49.5; // mph

    this->state = "KL";

    this->too_close = false;
    
    this->vehicle_gap_factor = 1;
}

Vehicle::~Vehicle() {}

// TODO - Implement this method.
void Vehicle::update_state(vector<vector<double>> sensor_fusion, vector<double> previous_path_x, double car_s, double end_path_s) {
	/*
    Updates the "state" of the vehicle by assigning one of the
    following values to 'self.state':

    "KL" - Keep Lane
     - The vehicle will attempt to drive its target speed, unless there is 
       traffic in front of it, in which case it will slow down.

    "LCL" or "LCR" - Lane Change Left / Right
     - The vehicle will IMMEDIATELY change lanes and then follow longitudinal
       behavior for the "KL" state in the new lane.

    "PLCL" or "PLCR" - Prepare for Lane Change Left / Right
     - The vehicle will find the nearest vehicle in the adjacent lane which is
       BEHIND itself and will adjust speed to try to get behind that vehicle.

    INPUTS
    - predictions 
    A dictionary. The keys are ids of other vehicles and the values are arrays
    where each entry corresponds to the vehicle's predicted location at the 
    corresponding timestep. The FIRST element in the array gives the vehicle's
    current position. Example (showing a car with id 3 moving at 2 m/s):

    {
      3 : [
        {"s" : 4, "lane": 0},
        {"s" : 6, "lane": 0},
        {"s" : 8, "lane": 0},
        {"s" : 10, "lane": 0},
      ]
    }

    */
    vector<string> possible_states;
    double cost_KL, cost_LCL, cost_LCR;
    
    cost_KL = cost_keep_lane(sensor_fusion, previous_path_x, car_s, end_path_s);
    cost_LCL = cost_lane_change(sensor_fusion, previous_path_x, car_s, end_path_s,"L");
    cost_LCR = cost_lane_change(sensor_fusion, previous_path_x, car_s, end_path_s,"R");
           
    if(this->lane == 0)
       possible_states = { "KL","LCR"};
    else if (this->lane == 1)
       possible_states = { "KL","LCL","LCR"};
    else if(this->lane == 2)
       possible_states = { "KL","LCL"};

    this->state = mincost_state(cost_KL, cost_LCL, cost_LCR, possible_states); 

}

vector<double> Vehicle::behavior_planning(vector<vector<double>> sensor_fusion, vector<double> previous_path_x, double car_s, double end_path_s) {


    /*
      Decides on state("KL"|"LCL"|"LCR") for ego vehicle transition to be made 
      based on cost (minimum is choosen)
    */

     update_state(sensor_fusion, previous_path_x, car_s, end_path_s);
     realize_state();
     cout << " my lane:" << this->lane << endl;
     
     return {this->lane, this->ego_vel};
}

double Vehicle::cost_keep_lane(vector<vector<double>> sensor_fusion, vector<double> previous_path_x, double car_s, double end_path_s) {
   
    /*
     Calculates cost involved being in state "KL"
     Summation of speed_cost(low when speed is near target speed)
     and desire_cost(low if it is safe to travel(at target speed) in this lane
     without collision)
     Range of speed_cost = 0 to 0.5
     Value of desire_cost is 0.1 or 0.6

     cost =  speed_cost + desire_cost  
    */
  
    double speed_cost = 0;
    double desire_cost = 0;
    double cost = 0;
    double dist_buffer = 35; // distance(meters) between ego vehicle and front vehicle  

    int prev_size = previous_path_x.size();
   
        
    if(prev_size > 0)
       car_s = end_path_s;
    
    for(int i=0; i < sensor_fusion.size(); i++) {
       
       float d = sensor_fusion[i][6];
  
       if(d < (2+4*lane+2) && d >(2+4*lane-2)) {
          
         double vx = sensor_fusion[i][3];
         double vy = sensor_fusion[i][4];
         double check_speed = sqrt((vx*vx)+(vy*vy));
         double check_car_s = sensor_fusion[i][5];
  
         check_car_s+=((double)prev_size*0.02*check_speed);
  
         if(this->ego_vel < target_speed+0.5) 
            speed_cost = STOP_COST*((target_speed -ego_vel)/target_speed);
         else
           speed_cost = 0.5;
        
         //corner cases after one round completion
         if((car_s > 6900.0 && car_s < 6945.554) && (check_car_s > 0 && check_car_s < 45)) {
            check_car_s += 6945.554;
         }
         if((check_car_s > car_s) && ((check_car_s - car_s) < dist_buffer))   {
           // if(check_speed < this->ego_vel) {
               this->too_close = true;       
               this->vehicle_gap_factor = (1- ((check_car_s - car_s)/dist_buffer));           // }    
          }
                  
         if(this->too_close == true)  
            desire_cost = 0.6;
         else  
            desire_cost = 0.1;
        }
    }
    cost = speed_cost + desire_cost;
    return cost;    
}


double Vehicle::cost_lane_change(vector<vector<double>> sensor_fusion, vector<double> previous_path_x, double car_s, double end_path_s, string direction) {

    /*
    Calculates cost involved in Lane change Left/Right
    cost = change_cost + desire_cost
    */
        
    double cost_violation  = 2.0;
    double cost_collison = 10.0;
    double available_room  = 40;//meters    
    double change_cost = 999;
    double desire_cost = 999;
    double cost = 999;
    
    
    if(this->too_close == true)
       desire_cost = 0.1;
    else
       desire_cost = 1.0;

    int prev_size = previous_path_x.size();
   
    if(prev_size > 0)
      car_s = end_path_s;
    
    if(direction == "R") { 
       if(this->lane == 2) {
          cost = cost_violation;
          return cost;
       }
    }

    if(direction == "L") {
       if(this->lane == 0) {
           cost = cost_violation;
           return cost;
       }
    }    

    double distance_nearest_front = 99999;
    double distance_nearest_behind = 99999;
    double veh_ID_nearest_front = NO_VEHICLE_ID; //999;
    double veh_ID_nearest_behind = NO_VEHICLE_ID; //999;
    
    int lane=this->lane;
  
    double car_s_backup = car_s;

    
    if(direction == "R") {    
       if((this->lane == 0) || (this->lane == 1))
           lane  = this->lane +1;  //pretend to be in this updated lane
    }
    if(direction == "L") {
       if((this->lane == 1) || (this->lane == 2))
           lane = this->lane - 1;
    }

    cout << "direction : " << direction << "sdc s: " << car_s << endl;    
    

    for(int i=0; i < sensor_fusion.size() ; i++) {
        //check traffic in future lane (left)
                          
        float d = sensor_fusion[i][6];
                        
        if( d < (2+4*lane+2) && d > (2+4*lane-2)) {
           //Left/Right lane vehicles data
           double vx = sensor_fusion[i][3];
           double vy = sensor_fusion[i][4];
           double check_speed = sqrt((vx*vx) + (vy*vy));
           double check_car_s = sensor_fusion[i][5];
           
           check_car_s+=((double)prev_size*0.02*check_speed);
           cout << " prev size " << prev_size << " check_speed: " << check_speed << " vx: " << vx << " vy: " << vy << endl; 
           cout << "s: "  << sensor_fusion[i][5] << " veh_ID: " << i ;                    cout << " check_car_s: " << check_car_s << "veh_ID: " << i << endl;        
           //get data of nearest front car and behind car in Left/Right lane 
         //corner cases near every round completion(when ego car is behind)
           if((car_s > 6900.0 && car_s < 6945.554) && (check_car_s > 0 && check_car_s < 45)) {
            check_car_s += 6945.554;
           }
                     
           if(check_car_s  >= car_s) {
              if((check_car_s - car_s) < distance_nearest_front){
                 distance_nearest_front = check_car_s - car_s;
                 veh_ID_nearest_front = i;
              }
           cout << " veh_ID_nearest_front: " << veh_ID_nearest_front << endl;
           }

           if((check_car_s > 6900.0 && car_s < 6945.554) && (car_s > 0 && car_s< 45)) {
             car_s += 6945.554;
           }

           if(check_car_s  < car_s) {
              if((car_s - check_car_s) < distance_nearest_behind){
                 distance_nearest_behind = car_s - check_car_s; 
                 veh_ID_nearest_behind = i;
              }
           cout << " veh_ID_nearest_behind: " <<  veh_ID_nearest_behind << endl;
           }
       }
    }
    cout << " ID nearest_front: " << veh_ID_nearest_front << endl;
    cout << " ID nearest_behind: " << veh_ID_nearest_behind << endl;

    //double check_car_s_update;

    car_s = car_s_backup;

    if(veh_ID_nearest_front != NO_VEHICLE_ID) 
       if((car_s > 6900.0 && car_s < 6945.554) && (sensor_fusion[veh_ID_nearest_front][5] > 0 &&  sensor_fusion[veh_ID_nearest_front][5] < 45)) {
            sensor_fusion[veh_ID_nearest_front][5] += 6945.554;
       }

    if(veh_ID_nearest_behind != NO_VEHICLE_ID)
       if((sensor_fusion[veh_ID_nearest_behind][5] > 6900.0 &&  sensor_fusion[veh_ID_nearest_behind][5] < 6945.554) && (car_s > 0 && car_s< 45)) {
            car_s += 6945.554;
    }
    
    
    if(veh_ID_nearest_front !=NO_VEHICLE_ID) {

        if((sensor_fusion[veh_ID_nearest_front][5] - car_s) > available_room) {
            if(veh_ID_nearest_behind !=NO_VEHICLE_ID) {
                if((car_s - sensor_fusion[veh_ID_nearest_behind][5] > 30)){
                    cout << "distance gap behind :" << car_s - sensor_fusion[veh_ID_nearest_behind][5] << endl; 
                    cout << "distance gap front :" << sensor_fusion[veh_ID_nearest_front][5] -car_s << endl;               
                    change_cost = 0.1;
                }
             }
             else {
                 cout << "NO_VEHICLE_ID_BEHIND: " << veh_ID_nearest_behind << endl;
                 cout << "vehicle ID nearest front " << veh_ID_nearest_front << endl;
                 cout << "distance gap front 1: " << sensor_fusion[veh_ID_nearest_front][5] - car_s << endl;
                 change_cost = 0.1;//0.6
             }
         }
         else
            change_cost = cost_collison;
      }
      else {
        if(veh_ID_nearest_behind !=NO_VEHICLE_ID) { 
           if((car_s - sensor_fusion[veh_ID_nearest_behind][5] > 30)&&(this->ego_vel > 40)) 
              change_cost = 0.2;//1.0
           else
             change_cost = cost_collison;
        }
        else {
           
           cout << "when no vehicle in adjacent lane | ego vel= "  << this->ego_vel;
           cout << "NO_VEHICLE_ID_FRONT:" << veh_ID_nearest_front << endl;
           cout << "NO_VEHICLE_ID_BEHIND: " << veh_ID_nearest_behind << endl;
           change_cost = 0.1;//0.7;
        }
      } 
      
     cost = change_cost + desire_cost;
     //cout << "LC" << direction <<"  cost :" << cost << endl;

     return cost;
}


string Vehicle::mincost_state(double cost_KL, double cost_LCL, double cost_LCR, vector<string>  possible_states){

   /*
    Calculates minimum cost and returns corresponding state(the best state transition)
   */
   string state,best_state;
   double min_cost = 99999;
   vector<double> costs;
   for (auto i = possible_states.begin(); i != possible_states.end(); ++i)
    cout << *i << ' ';

  // cout << "possible states: "<< possible_states << endl;
   
   for( int i= 0; i < possible_states.size(); i++) {
      if(possible_states[i] == "KL")
         costs.push_back(cost_KL);
      if(possible_states[i] == "LCL")
         costs.push_back(cost_LCL);
      if(possible_states[i] == "LCR")
         costs.push_back(cost_LCR);
   }

   for (auto i = costs.begin(); i != costs.end(); ++i)
       cout << "cost: " << *i << ' ';

   for( int i=0; i < possible_states.size(); i++) {
        state = possible_states[i];
        double cost  = costs[i];
        if (cost < min_cost) {
            min_cost = cost;
            best_state = state;
        }
    }

    if(this->state != "KL")
       best_state = "KL";
   
    cout << "best_state: " << best_state << endl;
    
    return best_state; 

}


void Vehicle::realize_state() {
   
    /*
    Given a state, realize it by adjusting velocity and lane.
    Note - lane changes happen instantaneously.
    */
    string state = this->state;
    if(state.compare("KL") == 0)
    {
        realize_keep_lane();
    }
    else if(state.compare("LCL") == 0)
    {
    	realize_lane_change_left();
    }
    else if(state.compare("LCR") == 0)
    {
    	realize_lane_change_right();
    }
    
}

void Vehicle::realize_keep_lane() {
    if(this->too_close == true) {
       //this->ego_vel -= 0.112; //0.224;
       this->ego_vel -= (this->vehicle_gap_factor * 0.448);//0.324
       cout << "vehicle_gap_factor : " << this->vehicle_gap_factor << endl;
       this->too_close = false;
    }
    else if(this->ego_vel < target_speed) {
       //this->ego_vel += 0.112;//0.168;//0.224;
        this->ego_vel += ((target_speed - ego_vel)/target_speed)*0.224;

    }
}

void Vehicle::realize_lane_change_left() {
    this->lane -=1;
    cout << "lane change left" << endl;
}    

void Vehicle::realize_lane_change_right() {
    this->lane +=1;
    cout << "lane change right" << endl;
}




