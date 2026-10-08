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

### 1. kinematic bicycle model
State $x = [x,\ y,\ \theta,\ v]^T$, inputs $u = [u_{thr},\ \delta]^T$:

$$
\dot x = v\cos\theta,\qquad
\dot y = v\sin\theta,\qquad
\dot\theta = \frac{v}{L}\tan\delta,\qquad
\dot v = k_a\,u_{thr} - c_{drag}\,v^2 - c_{roll}\,v
$$

Forward Euler integration with step $\Delta t = 0.1$ s:

   
