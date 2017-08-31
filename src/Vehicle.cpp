#include <iostream>
#include "Vehicle.h"
#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <iterator>
#include <cassert>

#define STOP_COST 0.5

/**
 * Initializes Vehicle
 */
Vehicle::Vehicle() {

    this->lane = 1;
    //this->s = s;
    this->ego_vel = 0.0;
    //this->d = d;
    this->target_speed = 49.5; // mph

    this->state = "KL";

    this->too_close = false;
    
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
    cost_LCL = cost_lane_change_left(sensor_fusion, previous_path_x, car_s, end_path_s);
    cost_LCR = cost_lane_change_right(sensor_fusion, previous_path_x, car_s, end_path_s);
           
    if(this->lane == 0)
       possible_states = { "KL","LCR"};
    else if (this->lane == 1)
       possible_states = { "KL","LCL","LCR"};
    else if(this->lane == 2)
       possible_states = { "KL","LCL"};

    this->state = mincost(cost_KL, cost_LCL, cost_LCR, possible_states); 

}


void Vehicle::configure(vector<int> road_data) {
	/*
    Called by simulator before simulation begins. Sets various
    parameters which will impact the ego vehicle. 
    */
    target_speed = road_data[0];
    lanes_available = road_data[1];
}

vector<double> Vehicle::behavior_planning(vector<vector<double>> sensor_fusion, vector<double> previous_path_x, double car_s, double end_path_s) {


    /*
      Decides on state("KL"|"LCL"|"LCR") for ego vehicle transition to be made 
      based on cost (minimum is choosen)
    */
     //cout <<"car_s:" << car_s<< endl;
     update_state(sensor_fusion, previous_path_x, car_s, end_path_s);
     realize_state();
     cout << " my lane: " << this->lane;
     
     return {this->lane, this->ego_vel};
}

//void Vehicle::path_planning() {

    /*
      Takes state as input provided by behaviour_planning module
      and generates path points(x,y) for optimal ,smooth trajectory using spline
    */
//}

double Vehicle::cost_keep_lane(vector<vector<double>> sensor_fusion, vector<double> previous_path_x, double car_s, double end_path_s) {
   
    /*
     Calculates cost involved being in state "KL"
     Summation of speed_cost(low when speed is near target speed)
     and desire_cost(low if it is safe to travel(at target speed) in this lane
     without collision)
     Range of speed_cost = 0 to 0.5
     Value of desire_cost is 0 or 0.5

     cost =  speed_cost + desire_cost  
    */
  
    double speed_cost = 0;
    double desire_cost = 0;
    double cost = 0;
  
    int prev_size = previous_path_x.size();
   
    cout <<"target_speed:" << target_speed <<"," << "ego_vel:" << this->ego_vel<< endl;
    
    if(prev_size > 0)
       car_s = end_path_s;
    
    for(int i=0; i < sensor_fusion.size(); i++) {
       
       float d = sensor_fusion[i][6];
       //cout << "d:" << d << endl;
  
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
                    
         //cout << "KL speed_cost:" << speed_cost << endl;  
         
         if((check_car_s > car_s) && ((check_car_s - car_s) < 30))   {
            this->too_close = true;
            //assert(2+2 == 5);
          }
                  
         if((this->too_close == true) && (check_speed < this->ego_vel)) 
            desire_cost = 0.6;
         else 
            desire_cost = 0.1;
       }
    }
    cost = speed_cost + desire_cost;
    cout << "KL speed_cost: " << speed_cost << "," << "KL desire_cost: " << desire_cost <<"," << "KL cost: " << cost << endl;
    return cost;    
}

double Vehicle::cost_lane_change_right(vector<vector<double>> sensor_fusion, vector<double> previous_path_x, double car_s, double end_path_s) {

    /*
    Calculates cost involved in Lane change Left
    cost = change_cost + desire_cost
    */
    //vector<string> possible_states;   
    
    double cost_violation  = 2.0;
    double available_room  = 40;//meters    
    double change_cost = 999;
    double desire_cost = 999;
    double cost = 999;

    //cout <<" Inside cost_lane_change_left" << endl;

    
    if(this->too_close == true)
       desire_cost = 0.6;
       // desire_cost = 0.0;
    else
       desire_cost = 1.0;
   
 
    //cout << "Inside LCL desire_cost: " << desire_cost << endl;
    
    //for lane change left is feasible
    //generate_trajectory_LCL();

    int prev_size = previous_path_x.size();
   
    if(prev_size > 0)
      car_s = end_path_s;
 
    if(this->lane == 2) {
       cost = cost_violation;
       return cost;
    }
    
    double distance_nearest_front = 99999;
    double distance_nearest_behind = 99999;
    double veh_ID_nearest_front = 999;
    double veh_ID_nearest_behind = 999;
    int lane=this->lane;

    int num_veh = 0;

    if((this->lane == 0) || (this->lane == 1))
        lane  = this->lane +1;
    
    for(int i=0; i < sensor_fusion.size() ; i++) {
        //check traffic in future lane (left)
                          
        float d = sensor_fusion[i][6];
        //cout <<"Inside LCL lane: " << lane << endl;
        //cout <<" Inside LCL d: " << d << endl;

        if( d < (2+4*lane+2) && d > (2+4*lane-2)) {
           //left lane vehicles data
           double vx = sensor_fusion[i][3];
           double vy = sensor_fusion[i][4];
           double check_speed = sqrt((vx*vx + vy+vy));
           double check_car_s = sensor_fusion[i][5];
           
           check_car_s+=((double)prev_size*0.02*check_speed);
           //cout << "Inside LCR check_car_s: " << check_car_s << "," <<"car_s :" << car_s<< endl;
       
           
           //get data of nearest front car and behind car in right lane 
                      
           if(check_car_s > car_s) {
              //nearest_front = check_car_s - car_s; 
              if((check_car_s - car_s) < distance_nearest_front){
                 distance_nearest_front = check_car_s - car_s;
                 veh_ID_nearest_front = i;
              }
           }

           if(check_car_s < car_s) {
              //nearest_behind = car_s - check_car_s;
              if((car_s - check_car_s) < distance_nearest_behind){
                 distance_nearest_behind = car_s - check_car_s; 
                 veh_ID_nearest_behind = i;
              }
           }
           num_veh++;
       }
       
     }
     /*
     if( num_veh ==0)
     {
        cout << "right lane is empty" << endl;
        change_cost =0.3;
        cost = change_cost;// + desire_cost;
        return cost;
     }
     */
     cout << "Nearest front vehicle ID: " << veh_ID_nearest_front << endl;
     cout << "Nearest behind vehicle ID: " << veh_ID_nearest_behind << endl;
     
     if(veh_ID_nearest_front !=999) {

        if((sensor_fusion[veh_ID_nearest_front][5] - car_s) > available_room) {
            if(veh_ID_nearest_behind !=999) {
                if((car_s - sensor_fusion[veh_ID_nearest_behind][5] > 30))                
                    change_cost = 0.2;
                else 
                    change_cost = 1.0;
           }
           else {
               if(this->ego_vel > 45)
                 change_cost = 0.3;
               else
                 change_cost = 0.7;
           }
        }
     }
     // else if((veh_ID_nearest_front ==999)&&(car_s - sensor_fusion[veh_ID_nearest_behind][5] > 30))
       // change_cost = 0.2;
    else if(veh_ID_nearest_front == 999) {
            if(veh_ID_nearest_behind == 999)
               change_cost=0.2;
            else { 
                if(veh_ID_nearest_behind !=999) 
                   if(car_s - sensor_fusion[veh_ID_nearest_behind][5] > 30)
                       change_cost = 0.2;
            } 
    }                          
/*
     if (((sensor_fusion[veh_ID_nearest_front][5] - car_s) > available_room)
         && (car_s - sensor_fusion[veh_ID_nearest_behind][5] > available_room)) 
             change_cost = 0;
     else
             change_cost = 0.8;
*/     
       
     cost = change_cost + desire_cost;
     cout << "LCR cost: " << cost << endl;

     return cost;
}



double Vehicle::cost_lane_change_left(vector<vector<double>> sensor_fusion, vector<double> previous_path_x, double car_s, double end_path_s) {

    /*
    Calculates cost involved in Lane change Left
    cost = change_cost + desire_cost
    */
    //vector<string> possible_states;   
    
    double cost_violation  = 2.0;
    double available_room  = 40;//meters    
    double change_cost = 999;
    double desire_cost = 999;
    double cost = 999;

    //cout <<" Inside cost_lane_change_left" << endl;

    
    if(this->too_close == true){
       desire_cost = 0.1;
       //assert(2+2 ==5);
    }
    else
       desire_cost = 0.8;
    

    //cout << "Inside LCL desire_cost: " << desire_cost << endl;
    
    //for lane change left is feasible
    //generate_trajectory_LCL();

    int prev_size = previous_path_x.size();
   
    if(prev_size > 0)
      car_s = end_path_s;
 
    if(this->lane == 0) {
       cost = cost_violation;
       return cost;
    }
    
    double distance_nearest_front = 99999;
    double distance_nearest_behind = 99999;
    double veh_ID_nearest_front = 999;
    double veh_ID_nearest_behind = 999;
    int lane=this->lane;

    int num_veh = 0;

    if((this->lane == 1) || (this->lane == 2))
        lane  = this->lane -1;
    
    for(int i=0; i < sensor_fusion.size() ; i++) {
        //check traffic in future lane (left)
                          
        float d = sensor_fusion[i][6];
        //cout <<"Inside LCL lane: " << lane << endl;
        //cout <<" Inside LCL d: " << d << endl;

        if( d < (2+4*lane+2) && d > (2+4*lane-2)) {
           //left lane vehicles data
           double vx = sensor_fusion[i][3];
           double vy = sensor_fusion[i][4];
           double check_speed = sqrt((vx*vx + vy+vy));
           double check_car_s = sensor_fusion[i][5];
           
           check_car_s+=((double)prev_size*0.02*check_speed);
           //cout << "Inside LCL check_car_s: " << check_car_s << "," <<"car_s :" << car_s<< endl;
       
           
           //get data of nearest front car and behind car in left lane 
                      
           if(check_car_s > car_s) {
              //nearest_front = check_car_s - car_s; 
              if((check_car_s - car_s) < distance_nearest_front){
                 distance_nearest_front = check_car_s - car_s;
                 veh_ID_nearest_front = i;
              }
           }

           if(check_car_s < car_s) {
              //nearest_behind = car_s - check_car_s;
              if((car_s - check_car_s) < distance_nearest_behind){
                 distance_nearest_behind = car_s - check_car_s; 
                 veh_ID_nearest_behind = i;
              }
           }
           num_veh++;
       }
       
     }
     /*
     if( num_veh ==0)
     {
        cout << "left lane is empty" << endl;
        change_cost =0.3+desire_cost;
        cost = change_cost;// + desire_cost;
        return cost;
     }
     */
     cout << "Nearest front vehicle ID: " << veh_ID_nearest_front << endl;
     cout << "Nearest behind vehicle ID: " << veh_ID_nearest_behind << endl;
     
     if(veh_ID_nearest_front !=999) {

        if((sensor_fusion[veh_ID_nearest_front][5] - car_s) > available_room) {
            if(veh_ID_nearest_behind !=999) {
                if((car_s - sensor_fusion[veh_ID_nearest_behind][5] > available_room))                
                    change_cost = 0.2;
                else 
                    change_cost = 1.0;
            }
            else {
               if(this->ego_vel > 45)
                change_cost =0.3;
              else
                 change_cost = 0.7;
           }
        }
     }
    else if(veh_ID_nearest_front == 999) {
            if(veh_ID_nearest_behind == 999)
               change_cost=0.2;
            else { 
                if(veh_ID_nearest_behind !=999) 
                   if(car_s - sensor_fusion[veh_ID_nearest_behind][5] > 30)
                       change_cost = 0.2;
            } 
    }                          
                    
/*
     if (((sensor_fusion[veh_ID_nearest_front][5] - car_s) > available_room)
         && (car_s - sensor_fusion[veh_ID_nearest_behind][5] > available_room)) 
             change_cost = 0;
     else
             change_cost = 0.8;
*/     
       
     cost = change_cost + desire_cost;
     cout << "LCL cost: " << cost << endl;

     return cost;
}

#if 0
int Vehicle::cost_lane_change_right(vector<vector<double>> sensor_fusion, vector<double> previous_path_x, double car_s, double end_path_s) {

    /*
    Calculates cost involved in Lane change Left
    cost = change_cost + desire_cost
    */
    //vector<string> possible_states;   
    
    double cost_violation  = 1.0;
    double cost = 999;
    double change_cost = 999;
    double desire_cost = 999;
    
    cout << "Inside LCR " << endl;    

    if(this->too_close == true)
       desire_cost = 0.2;
    else
       desire_cost = 0.4;
 
    
    //for lane change left is feasible
    //generate_trajectory_LCL();

    int prev_size = previous_path_x.size();
    double available_room = 60;  //meters  

    if(prev_size > 0)
      car_s = end_path_s;
 
    if(this->lane == 2) {
       cost = cost_violation;
       return cost;
    }
    
    double distance_nearest_front = 99999;
    double distance_nearest_behind = 99999;
    int veh_ID_nearest_front = 999;
    int veh_ID_nearest_behind = 999;
    
    for(int i=0; i < sensor_fusion.size() ; i++) {
        //check traffic in future lane (right)
        if((this->lane == 0) || (this->lane == 1)) 
           int lane = this->lane - 1;            
                   
        float d = sensor_fusion[i][6];
       

        if( d < (2+4*lane+2) && d > (2+4*lane-2)) {
           //right lane vehicles data
           double vx = sensor_fusion[i][3];
           double vy = sensor_fusion[i][4];
           double check_speed = sqrt((vx*vx + vy+vy));
           double check_car_s = sensor_fusion[i][5];
           

           //get data of nearest front car and behind car in right lane 
                      
           if(check_car_s > car_s) {
              
              //nearest_front = check_car_s - car_s; 
              if((check_car_s - car_s) < distance_nearest_front)
                 distance_nearest_front = check_car_s - car_s;
                 veh_ID_nearest_front = i;
           }

           if(check_car_s < car_s) {
              //nearest_behind = car_s - check_car_s;
              if((car_s - check_car_s) < distance_nearest_behind)
                 distance_nearest_behind = car_s - check_car_s; 
                 veh_ID_nearest_behind = i;
           }
       }
     }
     
     /*if (((sensor_fusion[veh_ID_nearest_front][5] - car_s) > available_room)
         && (car_s - sensor_fusion[veh_ID_nearest_behind][5] > available_room)) 
             change_cost = 0;
     else
             change_cost = 0.8;
     */
     cost = change_cost + desire_cost;

     return cost;
}
    
#endif

string Vehicle::mincost(double cost_KL, double cost_LCL, double cost_LCR, vector<string>  possible_states){

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
       cout << *i << ' ';

   for( int i=0; i < possible_states.size(); i++) {
        state = possible_states[i];
        double cost  = costs[i];
        if (cost < min_cost) {
            min_cost = cost;
            best_state = state;
        }
    }
   
     /*
    if(this->state == "LCL" && best_state == "LCR")
       best_state = "KL";
    if(this->state == "LCR" && best_state == "LCL")
       best_state = "KL";
    */
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
        cout << "inside realize_state:" << endl;
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
       //assert(2+2 ==5);
       this->ego_vel -=0.224;
       this->too_close = false;
    }
    else if(this->ego_vel < target_speed) {
       this->ego_vel +=0.224;
       //cout << "ego_vel: " << this->ego_vel << endl;
    }
}

void Vehicle::realize_lane_change_left() {
    this->lane -=1;
    this->ego_vel-=0.224;
}    

void Vehicle::realize_lane_change_right() {
    this->lane +=1;
    this->ego_vel-=0.224;
}




