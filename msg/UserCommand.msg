# The command for the robot specified by the user

StateLin3d       goal               # the goal state to reach
float64          t_left             # the time left to reach this state
int32            motion_type        # Type of Motion (Walk, Trott, Bound, Camel)
bool             motion_type_change # if the motion type changed from the previous message
bool             replay_trajectory  # Replay the already optimized trajectory in RVIZ
bool             use_solver_snopt   # otherwise Ipopt.
