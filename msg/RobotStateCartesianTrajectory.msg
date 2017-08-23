# The state of a robot expressed in the cartesian frame

# The header is used to specify the coordinate frame and the reference time for the trajectory durations
std_msgs/Header header

# A representation of a Cartesian trajectory
RobotStateCartesian[] points
