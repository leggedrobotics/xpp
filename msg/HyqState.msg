# The state of a robot
# This information must be completely sufficient to reconstruct the whole
# body state q, qd, qdd of the system.

BaseState              base            # The 6D pose of the base
sensor_msgs/JointState joints          # pos, vel, torque in each joint
float64[12]            joint_acc       # joint acceleration
bool[4]                ee_in_contact   # Defines for each ee if it is touching the environement