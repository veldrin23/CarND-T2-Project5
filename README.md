# CarND-Controls-MPC

## The model
I used the 'simple' Kinematic model that was discussed during the lessons. The model considers the vehicle on a 2D plane and takes into account the following: The...
* vehicles's position (x, y)
* vehicles's heading direction (psi)
* vehicles's velocity (v)
* vehicles's cross-track error - how far it's off from the ground truth (cte)
* vehicles's orientation error (epsi)

It ignores elements like the physics of the tyres on the road and whether the vehicle is on a decline/ incline. 

The model predicts the next state and the actuators (acceleration (a) and steering angle (delta))

The update equations are as follows: 

x[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt

y[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt

psi[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt

v[t] = v[t-1] + a[t-1] * dt

cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt

epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt



Using these state values, the model aims to predict the appropiate acceleration (a) and steering angle (delta), while keeping trying to minimize the error function. The error function takes the following into account:
* The square sum of cte and epsi (line 48-52 in MPC.cpp)
* The square sum of the difference actuators (line 55-58 in MPC.cpp)
* The square sum of the sequential actuators (line 61-65 in MPC.cpp)

The first error term is to penalize deviation from ground truth, while the last two is to penalize the actuators' behaviour (drastic sequential - and extreme values). I used scaling factors for these errors terms to smooth out the vehicle's behaviour to adjust for the error function.

## Timestep Length and Elapsed Duration
After playing around with a few values, I found that 7 timesteps with a elapse duration of 100ms keeps the vehicle nicely on track. I also tried using higher elapse duration values (.2, .15), but the vehicle reacted too slowly to keep a smooth ride. Higher values for the timestamp (9, 11) didn't have a huge impact on the vehicle's behaviour. 

## Polynomial Fitting and MPC Preprocessing
A third degree polynomial is fitted to the transformed waypoints. The polynomial sometimes 'flips' around the ground truth on straight parts of the road, but this has no negative impact on the vehicle's behaviour (thanks to the square sum of the sequential actuators error term). The coefficients are also used to calculated the cross track - and orientation errors.

## Model Predictive Control with Latency
To account for latecny, the state values are calculated with a delayed interval.  

## End result:

[![IMAGE ALT TEXT](http://img.youtube.com/vi/XA0P2HRMmi0/0.jpg)](http://www.youtube.com/watch?v=XA0P2HRMmi0 "CarND-T2-Project5")


