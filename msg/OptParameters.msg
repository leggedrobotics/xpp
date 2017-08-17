# Parameters used to generate this optimization/trajectory

geometry_msgs/Point[]   nominal_ee_pos    # nominal position of each endeffector
geometry_msgs/Vector3   ee_max_dev        # the maximum distance the endeffector can deviate from the nominal position

StateLin3d              goal_lin          # the linear state to reach (x,y,z and derivatives)
StateLin3d              goal_ang          # the angular state to reach (roll, pitch, yaw and derivatives)             

float64                 base_mass         # mass of the robot base (for plotting gravity force)