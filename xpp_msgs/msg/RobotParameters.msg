# Parameters used to generate this optimization/trajectory
# Should basically save class xpp::OptimizationParameters

# endeffector names (order of endeffectors, e.g. LF, RF, LH, RH)
string[] ee_names  

geometry_msgs/Point[]   nominal_ee_pos    # nominal position of each endeffector
geometry_msgs/Vector3   ee_max_rel_0        # the maximum distance the endeffector can deviate from the nominal position
geometry_msgs/Vector3   ee_max_rel_1        # the maximum distance the endeffector can deviate from the nominal position
geometry_msgs/Vector3   ee_max_rel_2        # the maximum distance the endeffector can deviate from the nominal position
geometry_msgs/Vector3   ee_max_rel_3        # the maximum distance the endeffector can deviate from the nominal position
geometry_msgs/Vector3   ee_min_rel_0        # the maximum distance the endeffector can deviate from the nominal position
geometry_msgs/Vector3   ee_min_rel_1        # the maximum distance the endeffector can deviate from the nominal position
geometry_msgs/Vector3   ee_min_rel_2        # the maximum distance the endeffector can deviate from the nominal position
geometry_msgs/Vector3   ee_min_rel_3        # the maximum distance the endeffector can deviate from the nominal position


float64                 base_mass         # mass of the robot base (for plotting gravity force)             
