# The common state of a robot used by joint or cartesian space

BaseState   base              # The 6D pose of the base
bool[]      ee_in_contact     # Defines for each ee if it is touching the environment
float64     t_global          # global time along trajectory
