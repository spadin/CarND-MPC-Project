# Writeup

## Model

The update equations are in the `FG_eval#operator`

I calculate the cost and store it into `fg[0]`. To calculate the cost I
multiply the variables that I felt were more important to minimize by a large
factor. cte and epsi are two such values. Meaning, if the cte or epsi increase
even slightly, the cost grows by a factor of 2000. The solvers goal is to
minimize the error.

This tries to minimize cte, epsi and v based on the reference state. Here I
multiply the cost of cte by 2000 and epsi by 500 because I'd like even small
differences of those to be minimized.

```
for(int i = 0; i < N; i++) {
  fg[0] += 2000*CppAD::pow(vars[cte_start + i] - ref_cte, 2);
  fg[0] += 500*CppAD::pow(vars[epsi_start + i] - ref_epsi, 2);
  fg[0] += CppAD::pow(vars[v_start + i] - ref_v, 2);
}
```

The following minimizes the use of the actuators.

```
for(int i = 0; i < N - 1; i++) {
  fg[0] += CppAD::pow(vars[delta_start + i], 2);
  fg[0] += CppAD::pow(vars[a_start + i], 2);
}
```

The following minimizes the values between sequential actuations. This helps
prevent jerky movements by the actuators.

```
for(int i = 0; i < N - 2; i++) {
  fg[0] += CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
  fg[0] += CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
}
```

Next, I initialize the model's initial state to the values in `vars`. Since the
cost is stored in `fg[0]` we have to shift where we store the subsequent values
by 1.

I calculate the next predicted state using the vehicle model equations provided
in the lessons.

```
  // Recall the equations for the model:
  // x_[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
  // y_[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
  // psi_[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
  // v_[t] = v[t-1] + a[t-1] * dt
  // cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
  // epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt
```

## Timestep Length and Elapsed Duration (N & dt)

Final choices for timestep length and elapsed duration.

```
size_t N = 10;
double dt = 0.1;
```

`N * dt` determine what's referred to as the "prediction horizon." By choosing
`N = 10` and `dt = 0.1` the model will be predicting 1 second into the future.
We don't really want this product to be larger than a couple of seconds, with
all the curves and changes to terrain, it doesn't make sense to predict so far
ahead into the future.

The two numbers are also related because `N / dt` gives you the number of steps
the will be calculated per prediction. In my case, with `N = 10` and `dt = 0.1`
the number of steps is 100.

Based on these factors, and the fact that the MPC walkthrough started with
these values, I also decided to start from these values. I tried modifying the
values to see if I could improve the performance, but I could not.

When I set `N` higher than 10, the car would run off the road right from the
start. The initial calculations that were being used to determine the steering
angle were not reliable.

When I set `dt` lower than 0.1, the calculations would be way off as soon as
the car was just getting started, causing the car to run off the road before it
even reached a speed of 10 mph. Setting `dt` higher than .1 made the first
calculation ahead of the car a little too far, which also made the car
eventually run off the road.

## Polynomial Fitting and MPC Preprocessing

In `main.cpp` I had to shift the `ptsx` and `ptsy` to the cars perspective.
Once that was done, I was able to get the coeffs using `polyfit` to fit the
points to a 3rd degree polynomial. A 3rd degree polynomial is good so curves in
the line are taken into account.

## Model Predictive Control with Latency

I confirmed my model was working by reducing the latency setting to
`this_thread::sleep_for(chrono::milliseconds(0));` This needs to be set to 100
to mimic real driving conditions, but for testing purposes it was helpful to
set it to 0.

Once I was confident the model was working I started compensating for latency
by changing `px` `py` `psi` and `v` before passing them into the solver. I
decided to send the solver the values the vehicle should have at the time the
solver was actually solving its problem. You can see this on lines 100-103 of
`main.cpp`

