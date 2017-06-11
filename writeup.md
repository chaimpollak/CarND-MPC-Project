# CarND MPC Project

The goal of the Model Predictive Control is to re-frame the task of following a trajectory as an optimization problem so that the solution to the optimization problem is the optimal trajectory.

A vehicle has 4 states:
* x
* y
* orientation (psi)
* velocity (v)


If we know the current state of the vehicle, can we tell where the vehicle will be in the future?

There are 2 main actuators, that given these actuators we can predict where the vehicle will be in the future:
* steering angle (delta)
* acceleration (a)

There is also one other measurement which we must take into consideration: 
* Lf 

It measures the distance between the front of the vehicle and its center of gravity. The larger the vehicle, the slower the turn rate.


The way we will predict where the vehicle will be at time t+1 is as follows:

```
x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
v_[t+1] = v[t] + a[t] * dt
cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
```

---------------------


Since the autonomous vehicle deals with a trajectory which was mapped out through a path planning algorithm (using localization), we will need to fit the trajectory into a 3rd degree polynomial (Third-degree polynomials are common since they can fit most roads) and predict the next state in time based on the fitted polynomial.

In our simulator, we are given the trajectory in map coordinates which we will need to convert to car coordinates before fitting the 3rd degree polynomial.
(see main.cpp line 100-120)

a [simple way to convert the waypoints into car coordinates](https://cdn-enterprise.discourse.org/udacity/uploads/default/original/4X/3/0/f/30f3d149c4365d9c395ed6103ecf993038b3d318.png) into car coordinates is as follows

```
for (int i = 0; i < ptsx.size(); i++) {

    double x_shifted = ptsx[i] - px;
    double y_shifted = ptsy[i] - py;

    points_x.push_back(x_shifted * cos(psi) + y_shifted * sin(psi));
    points_y.push_back(- x_shifted * sin(psi) + y_shifted * cos(psi));

}
```
ptsx and ptsy is waypoint locations in map coordinates


Once we have a reference trajectory (from the fitted polynomial), our goal is to minimize the error between the reference trajectory and the vehicle’s actual path, which is done using a cost function.

### Cost Function:

There are 2 main errors which we want to minimize
* Cross Track Error - Distance of vehicle from trajectory (cte)
* Orientation Error - Difference of vehicle orientation and trajectory orientation (epsi)

Ideally, we would like that both errors should be 0, so a simple solution will be to set our cost function as follows:
```
// fg[0] is the cost variable
fg[0] = 0;

for (int i = 0; i < N; i++) {
      // minimize cte
    fg[0] += (N-i)* CppAD::pow(vars[cte_start + i], 2);
      // minimize orientation error
    fg[0] += (N-i)*CppAD::pow(vars[epsi_start + i], 2);
}
```

Since N is the number of timesteps in the horizon, a reasonable assumption will be that we should care more about the closer points, hence `(N-i) * cost`.
 
But the problem is that if the total cost is 0 the vehicle will come to a stop. In order to solve this, we will add the velocity error in the cost function

```
fg[0] += CppAD::pow(vars[v_start + i] - ref_v, 2);
```

This will ensure that the vehicle is penalize for not maintaining the reference velocity.

But we still have one more problem - smoothness, we really don't want the vehicle should start jerking around the road every time we make a turn or speed up and down every so often. In order to solve this, we will add the control input magnitude and change rate to the cost function.

```
// minimize the steering angle
fg[0] += (N-i) * CppAD::pow(vars[delta_start + i], 2);
// minimize the value gap between sequential steering angles
fg[0] +=  CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
// minimize the acceleration 
fg[0] += (N-i) * CppAD::pow(vars[a_start + i], 2);
// minimize the value gap between sequential accelerations
fg[0] += CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
```

In order to value smoothness error over all other errors, we multiply the smoothness cost with some factor(see mpc.cpp line 67 - 90).


----------
### Latency:

In order to account for latency, I computed the state vector so that it is looking at 100 ms in the future
```
const double Lf = 2.67;
double dt = 0.1; // 100ms latency

double future_x = v * dt; // since x is zero we only need to account for the latency
double future_y = 0;
double future_psi = -v * steering_angle / Lf * dt;
double future_v = v + throttle * dt;  
// Cross-track error is the difference between the trajectory and the vehicle.
double cte = polyeval(poly_coeff, 0);
// Orientation Error
double epsi = -atan(poly_coeff(1));
state << future_x, future_y, future_psi, future_v, cte, epsi;
```

-------------
### Timestep Length, Elapsed Duration:

I chose the number of N as 10 and the number for dt as 0.05 (50 ms), since the product of N and dt will give us a half of a second into the horizon and given a reference speed of 30 mph and a latency of 100 ms, 500ms should be just right. I played around with many different values for N and dt (N in the range of [5, 25] and dt in the range of [0.001, 0.5] and found that N = 10 and dt = 0.05 is the most logical and performs best for a latancy of 100 ms).

