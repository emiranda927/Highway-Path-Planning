# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

## Project Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

## Simulator Data

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [`x`,`y`,`s`,`dx`,`dy`] values. `x` and `y` are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the `dx` and `dy` values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.
#### Main car's localization Data (No Noise)

`x` The car's x position in map coordinates
`y` The car's y position in map coordinates
`s` The car's s position in frenet coordinates
`d` The car's d position in frenet coordinates
`yaw` The car's yaw angle in the map
`speed` The car's speed in MPH

#### Previous path data given to the Planner 

`previous_path_x` The previous list of x points previously given to the simulator
`previous_path_y` The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

`end_path_s` The previous list's last point's frenet s value
`end_path_d` The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

sensor_fusion A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates]. 

#### Simulator Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Behavior and Path Planning

<img src="https://github.com/emiranda927/Highway-Path-Planning/blob/master/PathPlanningModel.JPG" width="615" />

### Path Planning Model
#### Path Horizon & Splines
The speed and path of the planner was defined by a horizon of points (`N`) that controlled both the path and speed of the trajectories. The spacing of the points within the path determined the speed while the time  between points was fixed at 0.02 seconds. The equation to calculate the number of points that defined this horizon is 'double N = (target_dist/(0.02 * ref_vel/2.24));'. 'target_x' is the target distance that we'd like the path to be and `ref_vel` is the target speed of the vehicle.

In order to ensure that the vehicle operates smoothly on a driveable path, I used a spline library to define a smooth path. After defining the spline based on the horizon, we use that spline to get points between each `N` horizon point. This is the path the ego vehicle takes.

#### Safety Flags
The first order of business was to ensure that the vehicle did not drive off-road or collide with other objects. For longitudinal safety, it was a simple matter of checking whether the ego_vehicle was withing 30 meters of a vehicle in front of it.

```
if(d < (2+4*lane+2) && d > (2+4*lane-2)) //if vehicle is within our current lane
  {
    if((check_car_s > car_s) && (check_car_s-car_s) < 30){ // if infront of ego_vehicle and w/in 30 meters
```
If those two checks are satisfied, we must slow down the ego vehicle to prevent a collision. This was accomplished with a "Too Close" flag and a vehicle speed decrement until the too close flag no longer applies.

I discuss lateral safety and safe states in more detail in the "Object Lists" and "Finite State Machine" section of this README.

#### Comfort

The original implementation of the longitudinal safety check caused the car to repeatedly speed up and slow down as it drifted into and out of that 30 meter safety range. In order to deal with this, I decided to decrease the decrement value within the 30 meter range and add another collision mitigation flag `FCW`, that increased the amount of braking if the vehicle drifted within 5 meters of the object infront of it. This provided a comfortable slow down without sacrificing the safety of the ego vehicle. 

### Behavior Planning Model
After addressing some safety concerns and proving out the path execution of my planner, it was time to give the vehicle some lateral motion smarts. There were many challenges, as well as opportunities for being creative during this part of development.

#### Object Lists
I quickly learned that telling the vehicle that it's safe to make a lane change is difficult to do while monitoring unfiltered data. The amount of if-statements and check flags grew exponentially as I discovered more and more edge scenarios that stalled out my planner. I eventually settled on restructuring my code to rely on fused object lists. These lists work similarly to the way we do as drivers. First, I defined a buffer zone around the vehicle in which it would be unsafe for the ego vehicle to move into that zone if there were objects around. These zones were seperated by lane for planning purposes (more on that later). Then, once an object entered into that zone, I added it to a "Close Objects List" for that specific lane. From there, it became a simple check to see if the zone was empty for that lane and allow the ego vehicle to generate a trajectory for entering into that zone.

I created a similar, more forward looking list in order to calculate average lane speeds, as well. This was used for the cost function portion of my code.

#### Finite State Machine
Now that lists containing objects of importance were defined, we could put a Finite State Machine to work. My FSM was relatively simple, only containing Keep Lane ("KL"), Prepare Lane Change, ("PLCL"/"PLCR") and Lane Change ("LCL"/"LCR") states. The program is intialized with a "KL" state and the cycle begins by passing this `ego_state` into a `successor_state` function, that proliferates a list of possible states from the current one. I used that list of successor states, in concert with my object lists, to determine which successor states were also `safe_states`. That is to say that the intended lane was free of objects and it is safe to expand that successor state. Once this list of safe states is created, we can begin calculating the best state for the ego vehicle to take.

#### Cost Considerations
The current state of this implementation only takes into account a few things I deemed important (other than safe manuevers, that is).

1. The number of nearby objects in a lane
2. The average velocity of nearby objects in a lane
3. Stagnation in Lane Keeping if other states are available

The summary of these costs are such that the ego vehicle values lanes that are free of other vehicles or have faster moving vehicles in that lane.

The costs of each lane are calculated using the previously defined object lists and lane speeds, the current lane, and the available safe states.

#### Trajectory Generation
In order to actually make any maneuver, we have to determine which lane is the best lane of available options. Using the calculated lane costs alone isn't sufficient for choosing a lane to maneuver into. For example, if the best lane available is the far right lane, but you are currently in the far left lane, it is probably not the best idea to cross over every lane of traffic right away. The `get_best_lane` function attempts to prevent actions like this from taking place. It uses the cost for each lane, the current lane of the ego_vehicle, and the intended lane of each safe state, to make a decision on where to go.

More simply, it automatically eliminates unsafe trajectories like lane skipping, and prioritizes motion over perpetual stagnation. I would run into issues where the best lane is associated with the lane to the left, for example, meaning that potential safe states are both prepare lane change left and lane change left, but the program needed a way to decide whether "PLCL" or "LCL" was the better manuever. The `get_best_lane` function prioritizes "LCL/R" over "PLCL/R", with the understanding that the lane change manuever can only exist given that a prepare lane change has already occured.

## Reflection and Next Steps
This project was tough in that defining an architecture or model is difficult upfront when you don't know what you'll be dealing with on the back-end. There were multiple times when I had to go back and modify a data structure or data handling method after discovering that there were issues with the planner. That being said, there is a lot of room for creativity and experimentation in a project like this.

In its current form, my cost function is pretty rudimentary. It works, but I'd like to add some other important considerations like object yaw rate to determine if other vehicles are making lane changes. I would also like to use yaw rates for some safety manuevers. In some chance encounters, vehicle from adjacent lanes would abruptly cut infront of the ego vehicle, causing a collision that couldn't be prevented. I'd like to implement a way of detecting these lane changes ahead of time and preventatively slowing down the ego vehicle to give it some breathing room.

I would also like to expand on the "Prepare Lane Change" states. In their current form, they really only serve as a safety check to make sure that the ego vehicle doesn't sit between lanes as it's deciding on lane change and that the intended lane is free. I would like it to also slow down and speed up the ego vehicle if it's boxed in by a slower moving vehicle infront and adjacent to it, so that it can get infront of the adjacent vehicle or get behind of the adjacent vehicle to make a lane change.
