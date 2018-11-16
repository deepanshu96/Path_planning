# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

## Model Documentation

* ### Prediction
In the first step the model deals with sensor fusion data provided by the car at each timestep in which the locations of other cars in left, right and center lane are determined to predict the future state of the ego vehicle. In this I used a vector container to keep the speeds of the car ahead and car adjacent in left and right lane to our ego vehicle. This was done to record the minimum speed amongst them in order to avoid collision for safe driving. Part of code mentioned below:-

```

            bool car_left, car_right, car_ahead;
            car_left = false;
            car_right = false;
            car_ahead = false;
            int car_ahead_cond, car_right_cond, car_left_cond;

            car_ahead_cond = car_right_cond = car_left_cond = 0;

            vector<pair<double, double> > speedlimit;
            for(int i=0;i<sensor_fusion.size();i++)
            {
                float d = sensor_fusion[i][6];
                if(d < (2+4*lane+2) && d > (2+4*lane-2) ) //same lane
                {
                    double vx = sensor_fusion[i][3];
                    double vy = sensor_fusion[i][4];
                    double check_speed = sqrt(vx*vx + vy*vy);
                    double check_car_s = sensor_fusion[i][5];
                    check_car_s += ((double) prev_size*0.02*check_speed);

                    if( (check_car_s > car_s) && ( (check_car_s - car_s) < 30) )
                    {
                        speedlimit.push_back(make_pair(check_car_s, check_speed));
                        car_ahead = true;
                        if((check_car_s-car_s) > 20 && car_ahead_cond<=1)
          				{
                            car_ahead_cond = 1;
          				}
          				else
          				{

                            car_ahead_cond = 2;

          				}

                    }

                }
                else if(d>0 && d> (2+4*lane-2-4) && d<(2+4*lane-2) && d<12 ) //left lane
                {
                    double vx = sensor_fusion[i][3];
                    double vy = sensor_fusion[i][4];
                    double check_speed = sqrt(vx*vx + vy*vy);
                    double check_car_s = sensor_fusion[i][5];
                    check_car_s += ((double) prev_size*0.02*check_speed);

                    if( (check_car_s >= car_s) && ( (check_car_s - car_s) < 30) )
                    {
                        if((check_car_s - car_s) < 10)
                            speedlimit.push_back(make_pair(check_car_s, check_speed));
                        car_left = true;
                        if((check_car_s-car_s) > 20 && car_ahead_cond<=1)
          				{
                            car_left_cond = 1;
          				}
          				else
          				{

                            car_left_cond = 2;

          				}
                    }
                    if(fabs(check_car_s - car_s)< 15)
                    {
                        car_left = true;
                    }
                }
                else if(d>0 && d<12 && d>(2+4*lane +2) && d<(2+4*lane+2+4)) //right lane
                {
                    double vx = sensor_fusion[i][3];
                    double vy = sensor_fusion[i][4];
                    double check_speed = sqrt(vx*vx + vy*vy);
                    double check_car_s = sensor_fusion[i][5];
                    check_car_s += ((double) prev_size*0.02*check_speed);

                    if( (check_car_s >= car_s) && ( (check_car_s - car_s) < 30) )
                    {
                        if((check_car_s - car_s) < 10)
                            speedlimit.push_back(make_pair(check_car_s, check_speed));
                        car_right = true;
                        if((check_car_s-car_s) > 20 && car_ahead_cond<=1)
          				{
                            car_right_cond = 1;
          				}
          				else
          				{

                            car_right_cond = 2;

          				}
                    }
                    if(fabs(check_car_s - car_s)< 15)
                    {
                        car_right = true;
                    }
                }
            }
            sort(speedlimit.begin(), speedlimit.end());
 ```

* ### Behaviour Planning
In this step the behaviour of ego vehicle was defined as to what next state and action it must choose given the current conditions obtained from the prediction step. The car moved in the adjacent lanes only if the vehicle ahead was going slow and at a certain minimum distance from our ego vehicle. Behaviour planning part of code mentioned below :-

```
 //Behaviour Planning
            if(car_ahead==true)
            {
                if(car_ahead_cond > 0)
                {
                    if(car_left==false && lane>0)
                    {
                         lane-=1;
                         ref_vel-=0.224;

                    }
                    else if(car_right==false && lane<2)
                    {
                         lane+=1;
                         ref_vel-=0.224;

                    }
                    else
                    {
                        if(car_ahead_cond==1)
                        {
                            double spd2 = speedlimit[0].second;
                            ref_vel = spd2*2.237;
                        }
                        else if(car_ahead_cond==2)
                        {
                            double spd2 = speedlimit[0].second;
                            ref_vel = spd2*2.237-5;
                        }
                    }
                }
            }

            speedlimit.clear();
```

* ### Trajectory generation
The trajectory generation step included generating a smooth trajectory for our vehicle using the spline function in order to reduce sudden jerk and high accelerations or breaking. The previous trajectory points generated not used in the new trajectory were used in order to get a smooth resulting trajectory. This was all mentioned in the q and a video provided in the project lessons. 

The car was able to drive 4.32 miles as mentioned in the project rubric and the below conditions were not violated in this period:-
* Car was always in the given speed limit. 
* Max acceleration and jerk were not exceeded.
* Car did not have a collision. 
* The car stays in its lane except for time between lane changing. 
* The car was smoothly able to change its lanes. 

The trajectory generation code is mentioned below :-

```
vector<double> ptsx;
          	vector<double> ptsy;

          	if(prev_size < 2)
          	{
          		double prev_car_x = car_x - cos(car_yaw);
          		double prev_car_y = car_y - sin(car_yaw);

          		ptsx.push_back(prev_car_x);
          		ptsx.push_back(car_x);

          		ptsy.push_back(prev_car_y);
          		ptsy.push_back(car_y);
          	}
          	else
          	{
          		ptsx.push_back(previous_path_x[prev_size-2]);
          		ptsx.push_back(previous_path_x[prev_size-1]);

          		ptsy.push_back(previous_path_y[prev_size-2]);
          		ptsy.push_back(previous_path_y[prev_size-1]);


          	}

          	vector<double> next_wp0 = getXY(car_s+30,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          	vector<double> next_wp1 = getXY(car_s+60,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          	vector<double> next_wp2 = getXY(car_s+90,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);

          	ptsx.push_back(next_wp0[0]);
          	ptsx.push_back(next_wp1[0]);
          	ptsx.push_back(next_wp2[0]);

          	ptsy.push_back(next_wp0[1]);
          	ptsy.push_back(next_wp1[1]);
          	ptsy.push_back(next_wp2[1]);


          	for (int i = 0; i < ptsx.size(); i++ )
          	{

          		//shift car reference angle to 0 degrees
          		double shift_x = ptsx[i]-ref_x;
          		double shift_y = ptsy[i]-ref_y;

				ptsx[i] = (shift_x *cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
				ptsy[i] = (shift_x *sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));

          	}


          	tk::spline s;


          	s.set_points(ptsx,ptsy);

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

          	for(int i = 0; i < previous_path_x.size(); i++)
          	{
          		next_x_vals.push_back(previous_path_x[i]);
          		next_y_vals.push_back(previous_path_y[i]);
          	}

          	double target_x = 30.0;
          	double target_y = s(target_x);
          	double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

          	double x_add_on = 0;

			for (int i = 1; i <= 50-previous_path_x.size(); i++) {

				if(ref_vel > car_speed)
				{
					car_speed+=.224;
				}
				else if(ref_vel < car_speed)
				{
					car_speed-=.224;
				}


				double N = (target_dist/(.02*car_speed/2.24));
				double x_point = x_add_on+(target_x)/N;
				double y_point = s(x_point);

				x_add_on = x_point;

				double x_ref = x_point;
				double y_ref = y_point;

				x_point = (x_ref *cos(ref_yaw)-y_ref*sin(ref_yaw));
				y_point = (x_ref *sin(ref_yaw)+y_ref*cos(ref_yaw));

				x_point += ref_x;
				y_point += ref_y;


				next_x_vals.push_back(x_point);
				next_y_vals.push_back(y_point);

```












   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

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

