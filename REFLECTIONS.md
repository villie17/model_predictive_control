# Reflections

Model Predictive Control Project implementation is discussed in detail below.

## The Model

* The state being tracked consists of 2D coordinates of car, speed, cross track error, error in desired steering angle. In code this is as follows
     state << px, py, psi, v, cte, epsi;

* The actuators are throttle reprenting acceleration(or decelaration) and steering angle. In code they are represented as
     a and delta

* Kinematic model is used for predicting future state. It follows the same outline as discussed in lesson. Kinematic model is represented as constraints 
  before passing them on to opimizer. Followin lines of code reprent the constraints resulting from Kinematic model
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 -(psi0 + (v0/Lf)*delta0 * dt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t] = cte1 - (f0 - y0) - (v0* CppAD::sin(epsi0) * dt);
      fg[1 + epsi_start + t] = epsi1 - (psi0 - psides0) -  ((v0/Lf)*delta0 * dt);

## Timestep Length and Elapsed Duration
* Timestep length N and elapsed duration dt are selected as 20 and 0.05 respectively. I tried with lower values of N but the car was slow to react the curves. Values higher than 20 also worked but 20 seemed to be good enough to keep car on course. For dt values higher than 0.05 forced the car to predict longer path which resulted in lower accuracy and car did not stay on track. For values lower than 0.05 the horizon was very small and the car became very erratic and veered off course quickly. 

## Polynomial fitting
* Before fitting the polynomial waypoints are mapped from global map coordinates to local vehicle coordinates.
 x = (x_map - x_car_map) * cos(psi) - (y_map - y_car_map) * sin(psi)
 y = (x_map - x_car_map) * sin(psi) + (y_map - y_car_map) * cos(psi)

After this polynomial is fit using the function provided. Some of waypoints are behind the car (-ve x). But they are not ignored.

## Latency
Latency of 100ms exists. To overcome affect of this latency, first output of solver at t=0 were discarded and the second one at t=1x0.05 seconds (i-e at t= 50ms) was used. This value was selected as it gave the best results. The later predictions at t=100ms were too off track to be helpful. In code 
   and 
   use_val = 1;
   solution.x[delta_start+use_val],   solution.x[a_start+use_val]


## Conclusion
The results of MPC are much smoother than PDI control and much better than what I could achieve with behavorial cloning in Term 1. 




Fields:

* `ptsx` (Array<float>) - The global x positions of the waypoints.
* `ptsy` (Array<float>) - The global y positions of the waypoints. This corresponds to the z coordinate in Unity
since y is the up-down direction.
* `psi` (float) - The orientation of the vehicle in **radians** converted from the Unity format to the standard format expected in most mathemetical functions (more details below).
* `psi_unity` (float) - The orientation of the vehicle in **radians**. This is an orientation commonly used in [navigation](https://en.wikipedia.org/wiki/Polar_coordinate_system#Position_and_navigation).
* `x` (float) - The global x position of the vehicle.
* `y` (float) - The global y position of the vehicle.
* `steering_angle` (float) - The current steering angle in **radians**.
* `throttle` (float) - The current throttle value [-1, 1].
* `speed` (float) - The current velocity in **mph**.


### `psi` and `psi_unity` representations

`psi`

```
//            90
//
//  180                   0/360
//
//            270
```


`psi_unity`

```
//            0/360
//
//  270                   90
//
//            180
```

