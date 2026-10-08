# control_project
The Bicycle Gym is a ROS 2 project in which an autonomous car is controlled to drive on a track using controllers  to regulate its speed and keep it on the track.

# Student information

Name: Omar Mohamed Mahmoud

ID: 2500479

Telephone no.: 01065877082

# System architecture 
The system packages are bicycle_sim, bicycle_control, and track_environment. bicycle_sim is used for the vehicle model and contains the simulator node (sim_node), RViz configuration and the launch file. bicycle_control it is used to choose which controller to implement (teleoperation bridge, longitudinal PID, velocity profiler, lateral PID, pure pursuit, MPC) 
