# control_project
The Bicycle Gym is a ROS 2 project in which an autonomous car is controlled to drive on a track using controllers  to regulate its speed and keep it on the track.

# Student information

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
so it makes the car settles at the required speed with small overshoot.

### 4.Velocity profiler










   
