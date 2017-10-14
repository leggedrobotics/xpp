# The state of a robot expressed in the Cartesian frame

duration                time_from_start   # global time along trajectory

# Position, velocity and acceleration of the base expressed in world frame
# The orientation quaternion maps base to world frame.
State6d                 base              # base pos/vel/acc in world

sensor_msgs/JointState  joint_state
bool[]                  ee_contact        # True if the foot is touching the environment



