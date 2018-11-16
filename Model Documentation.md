# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

The model used in this project mainly consisted of three steps namely prediction of other cars using sensor fusion data, defining behaviour of our ego vehicle and generating a trajectory for safe driving and lane changing.These are as mentioned below.

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
