# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

[//]: # (Image References)

[image1]: ./pics/behaviors.png "Behaviors"
[image2]: ./pics/distance_cost.png "Distance COst"
[image3]: ./pics/gap_cost.png "Gap Cost"
[image4]: ./pics/jmt.png "JMT"
[image5]: ./pics/result.png "Result"

In this project, the goal is to design a path planner that is able to create smooth, safe paths for the car to follow along a 3 lane highway with traffic. The path planner will be able to keep inside its lane, avoid hitting other cars and pass slower moving traffic by using localization, sensor fusion and map data.

## What is path planning?
A basic path planning is to produce a continuous motion that connects start and goal without collision with known obstacles. We need to know the map and the starting and goal point and also obstacles. The famous algorithms for path planning are “Breadth first search”, “Depth first search” , “Dijkstra” and “A star”.Those algorithms are discrete methods. In this projection, the world is continuous, so we need to use other algorithms. One choice will be “Hybrid A star”. This algorithm was implemented in “Junior” which won the second prize in the DARPA Urban Challenge, a robot competition organized by the U.S. Government.This method is mainly used in the unstructured environment. But in the highway setting, the environment is more structured(predefined rules regarding how to move on the road such as direction of traffic, lane boundaries and speed limits). Therefore other method will be needed.

So what will be the good algorithm for highway driving?

## Prediction
First step is to predict all dynamic objects(cars) on the road. In the intersection, the behavior of cars are variable(going straight, turn right and turn left with variable speed) so it will be pretty difficult to predict. In this setting we need to construct several models for each behavior. One method is [multiple-model algorithms for maneuvering target tracking](http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.61.9763&rep=rep1&type=pdf), which is “Model based method. Other methods are “Data driven” approach or “Hybrid model(model based and data driven)”. In this project, cars are on the highway, and they move on the same side of the road. Therefore, we can use very simplified process model to predict the cars behavior.

## Behavior Planning
The behavior of car in the highway can be classified as follows.
Basically, we drive constant speed and keep lane if there’s no car ahead of us. We will slow down the speed when the car in front is slow. If there’s enough space to change lanes in this situation, we will change lanes. We can only classify 3 behaviors:Keep lane(constant speed or slow down), lane change left and lane change right.

![Behaviors][image1]

How can we decide which behavior to choose?
The solution is to calculate cost functions for each behavior and choose the best one when it is safe and feasible.

## Cost functions
I set following cost functions to decide which behavior to choose.



### Distance Cost
The more distance ,the better score it will get.
![Distance Cost][image2]

### Gap Cost
The less gap between my car and the car in front, the more cost my car will get.
![Gap Cost][image3]

### Constant Cost
I added additional constant cost for different occasion.

1. Keep lane cost
If the gap in the ego lane is smaller than the adjacent lane, it will get additional constant cost to urge lane change.(+0.19)

2. Change lane cost
If the car try to change lane, it will get additional cost.This is intended to avoid collision due to lane change.(+0.20)

3. Enough gap cost
If there are enough gap in front, it will reduce the cost(-0.05)

4. Change lane with low speed
It the car try to lane change with low speed, it will get additional cost
since it will cause of collision.(+0.15)


After calculating the cost, I choose the best one but that may not be feasible. Therefore, I also set feasibility check.
Feasibility check

When “Lane Change” is chosen, I calculate the gap between my car and the target lane’s car behind. If the gap isn’t enough to change lane, the car will stay in the same lane(Keep lane).

Also, when “Keep Lane” is chosen and the gap between my car and the car in front isn’t enough, the car will slow down so that collision will be avoided.


## Trajectory Generation
Once we decide the behavior, we can predict the future position, velocity and acceleration. Also we already know the current position, velocity and acceleration. Therefore, we can get the trajectory from the current position to the predicted one. But just connecting the points aren’t enough, since that isn’t physically possible(It may result in infinite acceleration or at least it will be uncomfortable and dangerous for human). Avoiding discomfort will be must for self-driving car. So what discomfort us? That is jerk.
Therefore we need to get a trajectory that minimize jerk.
By calculating coefficients of following quintic equation, we can get the jerk minimizing trajectory. As you can see, there exists 6 coefficients and we have 6 boundary conditions. Thus we can get the coefficients.

![JMT][image4]

## Results
The result video is following.
![Result][image5]

## Future Work
There are a lot of things to improve the result.

1.Add multiple-model algorithms for maneuvering target tracking. Also adding data driven approach will enhance the result.

2. Modify the cost functions. There are constant cost in the current cost functions. Those may be improved by converting ones that considering speed of the car.

3. In this simulator, there’s no accidents from other cars. In the real world, we may be involved in the accidents. Therefore, adding more rules such as emergency stop for avoiding collisions that occurred in front or emergency speed up or lane change for avoiding collision from behind.

## Reference
[Reflections on Designing a Virtual Highway Path Planner (Part 1/3)](https://medium.com/@mithi/reflections-on-designing-a-virtual-highway-path-planner-part-1-3-937259164650)

[Path Planning in Highways for Autonomous Vehicle](https://medium.com/@mohankarthik/path-planning-in-highways-for-an-autonomous-vehicle-242b91e6387d)















### Simulator. You can download the Term3 Simulator BETA which contains the Path Planning Project from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).

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
