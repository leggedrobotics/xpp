# The state of a robot expressed in the cartesian frame

RobotStateCommon        common
StateLin3d[]            feet           # endeffector pos/vel/acc in world
geometry_msgs/Vector3[] ee_forces      # endeffector forces expressed in world
