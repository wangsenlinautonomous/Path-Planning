# Path Planning
## Overview
For path planning it mainly contains the following modules:
* Prediction
* Behavior planning
* Trajectory planning

<img src="https://user-images.githubusercontent.com/40875720/52897526-97d9a700-3210-11e9-9700-15c74bde2365.PNG" width="400">

## Prediction

The purpose of the prediction module is to predict the position of the other vehicles nearby.

<img src="https://user-images.githubusercontent.com/40875720/52897676-30bcf200-3212-11e9-8cec-7d6ed136fef0.PNG" width="400">

For this module, it contains the following steps:
* Go through all the vehicles which come from sensor fusion data
* Check which lane the vehilce is in
* Predict the position of the vehicle in the future
* Find out the sensitive vehicles (Ahead, left or right)

```
//Step 1: PREDICTION

/*
This module just find out whether there are some cars in front of the host vehicle, on the left and on the right 
of the vehicle, then output car_left, car_right and car_ahead.
In the real case, we need to think more about it. Like using Kalman filter to track the path of each vehicle
*/
bool car_left= false;
bool car_right = false;
bool car_ahead = false;
for(int i=0; i < sensor_fusion.size(); i++) 
{
							
  float d = sensor_fusion[i][6];
  int check_car_lane;
				
  if(d > 0 && d < 4) 
  {
    check_car_lane = 0;
  } 
  else if(d > 4 && d < 8) 
  {
    check_car_lane = 1;
  } 
  else if(d > 8 and d < 12) 
  {
    check_car_lane = 2;
	} 	
							
  double vx = sensor_fusion[i][3];
  double vy = sensor_fusion[i][4];
  double check_speed = sqrt(vx*vx+vy*vy);
	double check_car_s = sensor_fusion[i][5];	

  check_car_s += ((double)prev_size*0.02*check_speed);
	if(check_car_lane == lane) 
  {
							
    car_ahead |= check_car_s > car_s && (check_car_s - car_s) < 30;										

  } 
  else if((check_car_lane - lane) == -1) 
  {
    car_left |= (car_s+30) > check_car_s  && (car_s-30) < check_car_s;
  } 
  else if((check_car_lane - lane) == 1) 
  {
								
    car_right |= (car_s+30) > check_car_s  && (car_s-30) < check_car_s;
							
  }
}

```

## Behavior planning

The purpose of behavior planning is to plan the next step of the vehicle(turn left, turn righ, decelerate...)
In the real case, we should think more about this moudle, such as the cost function the state machine and so on. But in my case, I did not think much about it.
I only make the judgement according to the status of the host vehicle(Has ahead vehilce, left vehilce or right vehilce)

<img src="https://user-images.githubusercontent.com/40875720/52898053-28fe4d00-3214-11e9-87dc-868f269d9005.PNG" width="400">

```
//Step 2:BEHAVIOUR
/*
This module is used to determine the behaviour of the host vehilce according to the different situations.
In the real case, we need to consider the cost functions also the state machine to make the final decision
*/
if(car_ahead) 
{
  if(!car_left && lane > 0) 
  {
    lane--;
  } 
  else if(!car_right && lane !=2) 
  {
    lane++;
  } 
  else if(!car_left && lane !=2) 
  {
    lane++;
  }
  else 
  {
    ref_vel -= speed_diff;
  }
} 
else if(ref_vel < max_accel)
{
  ref_vel += speed_diff;
}
```

## Trajectory planning

The purpose of trajectory planning is to generate waypoints for the host vehilce.
The basic mehtod is showing as the below pic

<img src="https://user-images.githubusercontent.com/40875720/52898144-644d4b80-3215-11e9-9abb-16754ae84536.PNG" width="400">


```						
double target_x = 30.0;
double target_y = s(target_x);
double target_dist = sqrt(target_x*target_x + target_y*target_y);

double x_add_on = 0;

for( int i = 1; i < 50 - prev_size; i++ ) 
{
              
  double N = target_dist/(0.02*ref_vel/2.24);
  double x_point = x_add_on + target_x/N;
  double y_point = s(x_point);

  x_add_on = x_point;

  double x_ref = x_point;
  double y_ref = y_point;

  x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
  y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

  x_point += ref_x;
  y_point += ref_y;

  next_x_vals.push_back(x_point);
  next_y_vals.push_back(y_point);
}
```
