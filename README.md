# control_project
The Bicycle Gym is a ROS 2 project in which an autonomous car is controlled to drive on a track using controllers  to regulate its speed and keep it on the track.

## Student information

Name: Omar Mohamed Mahmoud

ID: 2500479

Telephone no.: 01065877082

# System architecture 
The system packages are bicycle_sim, bicycle_control, and track_environment. bicycle_sim is used for the vehicle model and contains the simulator node (sim_node), RViz configuration and the launch file. bicycle_control it is used to choose which controller to implement (teleoperation bridge, longitudinal PID, velocity profiler, lateral PID, pure pursuit, MPC) 

the system nodes are teleop_twist_keyboard, teleop_bridge, kinematic_bicycle, controller, path_gen, lap_analyzer,rqt_plot/plotjugger/RViz

The system topics are throttle, steer, state, path, cmd_vel, lap/metrics 

# Mathematical equations  

### 1. Kinematic bicycle model
State $x = [x,\ y,\ \theta,\ v]^T$, inputs $u = [u_{thr},\ \delta]^T$:

$$
\dot x = v\cos\theta,\qquad
\dot y = v\sin\theta,\qquad
\dot\theta = \frac{v}{L}\tan\delta,\qquad
\dot v = k_a\,u_{thr} - c_{drag}\,v^2 - c_{roll}\,v
$$

Forward Euler integration with step $\Delta t = 0.1$ s:

$$
x_{k+1} = x_k + \dot x_k\,\Delta t
$$
 
After which the heading is wrapped, and the speed is clamped to its maximum. So, at this step, the states of the car are its coordinates (x,y), its heading angle and its speed. It also has the throttle and steering angle as inputs. 

### 2.Teleoperation bridge

It is an open-loop control. gets its data from the twist-stamped message. It converts the inputs from the keyboard to the throttle and steering angle and maps them into the range of the throttle (-1,1) and maximum steering angles.

$$
u_{thr} = \mathrm{clip}\left(\frac{v_{cmd}}{5.0}, -1, 1\right),\qquad
\delta = \mathrm{clip}\left(\frac{\omega_{cmd}}{1.0}, -\delta_{max}, \delta_{max}\right)
$$

A **watchdog** runs at 10 Hz: if the last Twist is older than 0.5 s, throttle and steering are set to zero.
(Zero throttle is *not* braking, so the car coasts down slowly.)

### 3.Longitudinal PID with anti-windup
first we need to calculate the speed error $e_v = v_{target} - v$:

$$
u_{thr} = \mathrm{clip}\Big(K_p e_v + K_i \int e_v\,dt + K_d \frac{de_v}{dt},\ -1,\ 1\Big)
$$

Gains: $K_p = 1.0$, $K_i = 0.2$, $K_d = 0.05$. **Anti-windup:** the integral state is clamped to
$\pm 2.0$, so it cannot grow without bound while the throttle is saturated. Negative output brakes the car.
So it makes the car settle at the required speed with small overshoot.

### 4. Velocity profiler

Path curvature $\kappa$ is estimated from the heading change over the neighbouring waypoints,
$\kappa \approx \Delta\psi / \Delta s$. The speed limit comes from a lateral-acceleration bound
$a_{lat,max} = 5\ \mathrm{m/s^2}$:

$$
v_{max} = \min\!\left(\sqrt{\frac{a_{lat,max}}{|\kappa|}},\ v_{cap}\right),\qquad v_{cap} = 7.5\ \mathrm{m/s}
$$

If the curvature is unavailable, the default speed (4.0 m/s) is used. This target feeds the longitudinal PID, so this makes the car drive well on sharp turns or on zig-zag roads at the highest available speed.

### 5. Cross-track error and heading error
For the nearest path segment, the signed CTE is positive when the car is left of the path (sign of the cross product $d_x(y-y_1) - d_y(x-x_1)$), and the heading error is $\psi_{err} = \mathrm{wrap}(\theta - \psi_{path})$.

### 6. Lateral PID

Steering from CTE and heading error. Since a car to the left (positive CTE or heading error) must steer right
(negative angle), the whole sum is negated:
 
$$
\delta = -\Big(K_p\,e_{ct} + K_i\!\int e_{ct}\,dt + K_d\,\dot e_{ct} + K_\psi\,\psi_{err}\Big)
$$
 
with $K_p = 0.8,\ K_i = 0.02,\ K_d = 0.15,\ K_\psi = 0.5$, integral clamped to $\pm 1$, and the output clipped to
$\pm 35^\circ$.

This equation makes the car track the road, move on it and correct its heading angle so it minimises its lateral error.

### 7. Pure Pursuit

Adaptive lookahead: $L_d = \mathrm{clip}(k_v v + l_{min},\ l_{min},\ l_{max})$ with $k_v = 0.25$,
$l_{min} = 0.8$ m, $l_{max} = 2.5$ m. The target point is the first waypoint, walking forward from the nearest one,
at least $L_d$ metres away. After transforming it into the vehicle frame ($x$ forward, $y$ left),

$$
\alpha = \mathrm{atan2}(y_{local},\ x_{local}),\qquad
\delta = \mathrm{atan2}\left(2L\sin\alpha,\ L_d\right)
$$

This gives the car the distance ahead to drive to it, so this equation calculates how to go to the lookahead distance as fast as possible.

### 8. Extended kinematic MPC

At every step a nonlinear program is solved over $N = 10$ steps ($\Delta t = 0.1$ s) for the inputs
$u = [\delta_0, a_0, \dots, \delta_{N-1}, a_{N-1}]$, using the same bicycle model with speed as a state
($v \leftarrow v + a\,\Delta t$). Tracking errors are projected into the path-aligned (Frenet) frame of each reference
point $(x_r, y_r, \psi_r, v_r)$:

$$e_{lat} = -\sin\psi_r\,(x-x_r) + \cos\psi_r\,(y-y_r),\qquad e_{lon} = \cos\psi_r\,(x-x_r) + \sin\psi_r\,(y-y_r)$$

$$J = \sum_{k=0}^{N-1} w_{lat}e_{lat}^2 + w_{lon}e_{lon}^2 + w_{\psi}e_{\psi}^2 + w_v e_v^2 + w_\delta\delta_k^2 + w_{\Delta\delta}(\delta_k-\delta_{k-1})^2 + w_a a_k^2$$

Weights: $w_{lat}=30,\ w_{lon}=1,\ w_\psi=10,\ w_v=1,\ w_\delta=0.2,\ w_{\Delta\delta}=6,\ w_a=0.1$.
Bounds: $\vert{}\delta\vert{} \le 35^\circ$, $\vert{}a\vert{} \le k_a$. The solver is SciPy SLSQP (25 iterations, `ftol` 1e-3) with a
**warm start** from the previous solution shifted by one step. Only the first control is applied
(**receding horizon**), and the throttle is $u_{thr} = a_0 / k_a$.

### 9.  Lap analyzer

The analyzer projects the car onto the path, accumulates |CTE| and speed for each lap, and detects a lap when the
progress along the path wraps from the last quarter to the first quarter of the track. It prints, per lap: lap time,
mean / RMS / max CTE, mean / max speed and total distance. It also publishes live telemetry (see the topic table)
and draws a CTE whisker (green → red) and a text HUD in RViz.

## Benchmark

Each autonomous controller ran for at least three full laps with the lap analyzer. Table rule: **best lap** =
fastest lap, **top speed** = highest speed in any lap, **CTE mean / RMS** = averages over the laps, **max CTE** =
worst lap. Lateral PID and Pure Pursuit used the curvature-based speed profile (up to 7.5 m/s); the MPC branch of
the provided controller node uses a constant 4.0 m/s speed reference. Manual driving used the keyboard with the
cruise controller on.

### leaderboard 

| Controller | Best lap (s) | Top speed (m/s) | Mean CTE (m) | RMS CTE (m) | Max CTE (m) | Laps / status |
|---|---|---|---|---|---|---|
| Manual teleoperation | 149.00 | 13.14 ¹ | 1.13 | 1.58 | 8.76 | 3 laps over 2 sessions, with off-track excursions |
| Lateral PID (reactive) | 81.10 | 7.62 | 0.40 | 0.56 | 3.08 | 3 |
| Pure Pursuit (preview) | 71.99 | 7.41 | 0.042 | 0.064 | 0.385 | 5 |
| Extended kinematic MPC (fixed 4 m/s) | 121.80 | 3.94 | 0.073 | 0.098 | 0.368 | 3 |
| *Pure Pursuit, fixed 4 m/s (supporting run)* | *111.20* | *4.29* | *0.026* | *0.050* | *0.331* | *2* |
 
¹ A one-off peak when the keyboard speed setting was raised too high; the cleanest manual lap peaked at 5.19 m/s.
 
### 5.3 Per-lap data
 
**Lateral PID**
 
| Lap | Time (s) | CTE mean / RMS / max (m) | Speed mean / max (m/s) | Distance (m) |
|---|---|---|---|---|
| 1 | 82.10 | 0.381 / 0.537 / 2.236 | 5.90 / 7.62 | 483.9 |
| 2 | 81.10 | 0.370 / 0.505 / 1.877 | 5.96 / 7.51 | 483.8 |
| 3 | 81.57 | 0.446 / 0.646 / 3.082 | 6.02 / 7.49 | 491.6 |
 
**Pure Pursuit (curvature profile)**
 
| Lap | Time (s) | CTE mean / RMS / max (m) | Speed mean / max (m/s) |
|---|---|---|---|
| 1 | 73.10 | 0.040 / 0.061 / 0.356 | 6.10 / 7.41 |
| 2 | 71.99 | 0.043 / 0.066 / 0.357 | 6.18 / 7.40 |
| 3 | 72.30 | 0.043 / 0.064 / 0.383 | 6.16 / 7.40 |
| 4 | 72.30 | 0.043 / 0.065 / 0.385 | 6.16 / 7.40 |
| 5 | 72.51 | 0.042 / 0.062 / 0.384 | 6.14 / 7.40 |
 
**MPC (fixed 4 m/s)**
 
| Lap | Time (s) | CTE mean / RMS / max (m) | Speed mean / max (m/s) |
|---|---|---|---|
| 1 | 122.40 | 0.073 / 0.098 / 0.340 | 3.66 / 3.93 |
| 2 | 121.80 | 0.074 / 0.101 / 0.362 | 3.67 / 3.94 |
| 3 | 121.80 | 0.071 / 0.094 / 0.368 | 3.68 / 3.93 |
 
**Pure Pursuit (fixed 4 m/s, supporting run)**
 
| Lap | Time (s) | CTE mean / RMS / max (m) | Speed mean / max (m/s) |
|---|---|---|---|
| 1 | 111.30 | 0.026 / 0.050 / 0.308 | 4.00 / 4.29 |
| 2 | 111.20 | 0.025 / 0.049 / 0.331 | 4.00 / 4.00 |
 
**Manual teleoperation**
 
| Lap | Time (s) | CTE mean / RMS / max (m) | Speed mean / max (m/s) | Distance (m) |
|---|---|---|---|---|
| Session 1, lap 1 | 303.30 | 1.697 / 2.176 / 8.755 | 1.53 / 13.14 | 464.8 |
| Session 2, lap 1 | 264.90 | 0.782 / 1.308 / 5.021 | 1.72 / 3.99 | 455.6 |
| Session 2, lap 2 | 149.00 | 0.920 / 1.245 / 3.938 | 3.02 / 5.19 | 449.4 |

The first lap of every session contains the idle time before the first command, so its time and mean speed are
not representative.

## controllers comparison

* **lateral PID** is the simplest and cheapest controller, but it is reactive, which means it only reacts to the error that has already happened and exists (it makes an error, then corrects it). Also, it has no vision for the road ahead, so it had the worst tracking among the controllers (mean CTE ≈ 0.40 m, max 3.08 m).

* **pure pursuit** has the best tracking and the fastest laps (mean CTE ≈ 0.042 m, 72 s per lap). Its lookahead point averages out the waypoint noise and gives a smooth movement along the path. Its disadvantage is that it is purely geometric, so it ignores the dynamics and actuator limits.

* **MPC** is the most capable in theory and the most expensive to compute. It had the lowest maximum CTE. On the other hand, its mean and RMS error were higher than pure pursuit even when they had the same speed 4m/s (from the table above).

* **Manual driving** was the worst at all and; it had the largest errors and exited the path many times.

**Important note: ** the MPC and the pure pursuit didn't have the same speed, so I made another test in which the pure pursuit drive with 4m/s, but pure pursuit also tracked the track better than MPC.

## Why MPC should track better than pure pursuit and lateral PID 



 

























   
