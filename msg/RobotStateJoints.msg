# The state of a robot
# This information must be completely sufficient to reconstruct the whole
# body state q, qd, qdd of the system.

RobotStateCommon       common
sensor_msgs/JointState joints            # pos, vel, torque in each joint
float64[]              joint_acc         # joint acceleration
int32                  n_joints_per_ee   # number of joints per endeffector