# Parameters used to generate this optimization/trajectory
# Should basically save class xpp::OptimizationParameters

# endeffector names (order of endeffectors, e.g. LF, RF, LH, RH)
string[] ee_names  

geometry_msgs/Point[]   nominal_ee_pos    # nominal position of each endeffector
geometry_msgs/Vector3   ee_max_dev        # the maximum distance the endeffector can deviate from the nominal position

float64                 base_mass         # mass of the robot base (for plotting gravity force)             
