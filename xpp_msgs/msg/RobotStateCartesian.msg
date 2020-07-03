# The state of a robot expressed in the Cartesian frame

duration                time_from_start   # global time along trajectory

# Position, velocity and acceleration of the base expressed in world frame
# The orientation quaternion maps base to world frame.
State6d                 base              # base pos/vel/acc in world

StateLin3d[]            ee_motion         # endeffector pos/vel/acc in world
geometry_msgs/Vector3[] ee_forces         # endeffector forces expressed in world
geometry_msgs/Vector3[] ee_decision_         # endeffector forces expressed in world
geometry_msgs/Vector3[] ee_f_c_1         # endeffector forces expressed in world
geometry_msgs/Vector3[] ee_f_c_2         # endeffector forces expressed in world
geometry_msgs/Vector3[] ee_vel_loc_         # endeffector in local frame
bool[]                  ee_contact        # True if the foot is touching the environment



