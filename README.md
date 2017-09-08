# Path Planning
In this project, designed a path planner that is able to create smooth, safe paths for the car to follow along a 3 lane highway with traffic. It also ensures self driving car to keep inside its lane, avoid hitting other cars, and pass slower moving traffic all by using localization, sensor fusion, and map data.

**Note:** Please use updated CMakeLists.txt and files from "src" folder

The steps involved in designing path planner

i) **Behavior planner :** - Provides guidance to trajectory planner about what sorts of maneuvers they should plan trajectories for. (like keep lane, lane change left, lane change right)

* **Cost functions**: Designed cost functions for keep lane, change lane and decision on ego vehicle behaviour is based on lower cost.
 FSM of 3 states are used: "KL" , "LCL" and "LCR". 
 
   ***KL cost function:*** Speed cost and desire cost are the values calculated on the basis of: 
   
       speed cost varies from 0 to 0.5(STOP_COST) and desire_cost is 0.1 or 0.6
       i.e speed cost decreases when ego vehicle speed approaches toward speed limit 
       and increases when slowing down.
      
       desire cost is when front vehicle and ego vehicle are coming nearer within  buffer zone, 
       desire cost is elevated from 0.1 to 0.6. i.e cost increases when ego vehicle desires to continue
       in this state("KL").  
            
    ***LC cost function:*** changing cost and desire cost are the values calculated on the basis of:
    
      desire cost: The cost involved by desiring to be in this state("LCL" or "LCR"). i.e when ego vehicle is too 
      close to front vehicle, the desire cost of lane change is reduced so that it favours lane changing.
      
      changing cost: This is cost involved when ego vehicle wants to change to this state. 
      When there is chances of collision is more when lane change occurs the changing cost is 
      elevated so that penalizing the lane change.
      
       

ii) **Trajectory planner :** - Generates drivable trajectories with ensuring max acceleration in m/s^2 and Jerk in m/s^3 are not exceeded, smooth changing of lanes to pass over the slower moving front vehicle in order to maintain less or equal to speed limit (49.5 mph).

The trajectory generation uses spline technique and same implementation from walk through is adapted.


### Valid Trajectories

 ![](lane_change.jpg)

The above screenshot shows the smooth changeover of lane with details on acceleration and jerk within limits , run beyond 1 lap and also speed limit of 49.5 mph.

The sample video of run is available https://youtu.be/aHAss-NvcbU
It is also tested multiple times and ran beyond 15 miles without an incident.

**Challenges and solutions**

i) ***Speed control***: When ego vehicle approaches near the front vehicle within buffer zone of 35 meters, the speed has to be reduced. Intially tried by tuning velocity variation with fixed value of 0.224 and other fixed values but resulted ego vehicle moving back and forth repeatedly till change lane to pass over the vehicle instead of ego vehicle following at speed of front vehicle till lane change occurs.

To realize,the logic used is the ego vehicle velocity decreases rapidly(more nearer then more speed reduction) when appraches front vehicle within buffer zone to avoid collision. Implemented in the following function.

*Vehicle::realize_keep_lane()*

This implemetation effectively handles by avoiding collision in situations like when suddenly a vehicle from adjacent lane changes lane and  comes within buffer zone in front of ego vehicle.

ii) ***Vehicles unidentified***: These corner cases occur(when completing lap) i.e when ego vehicle and other vehicles position (s)in lanes are divided on either side of  6945.544 meters.Solution for this is implemented in function *Vehicle::cost_keep_lane* (line 143-145) and *Vehicle::cost_lane_change* (line 241-243 , 253-255 etc)

iii) ***smooth trajectory***: when lane change occurs, faced problem with max jerk warning and in order to avoid this used spline technique explained in walkthrough and also number of way points redued from 50 to 30 in order to handle dynamic scenarios while tackling errant behavior of other cars in traffic.

### Reflection ###

This project demanded significant time to meet the rubric points. Had to work much on cost function design and tuning for behaviour planning of ego vehicle.

Had to run multiple times in simulator to find out the corner cases :

like under different traffic conditions, handling errant behaviour of other cars sometimes and also nearing the lap finishing the front vehicles have lower value 's' than the behind vehicle(i.e before they reach 6945.544m).

Max jerk is handled nicely by spline technique learned from walkthorugh.

Implementation of optimized lane changing, this means that the car only changes into a lane that improves its forward progress is future work.

The below content is Udacity project description

# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

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

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

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

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

