# README #

This repo contains ros messages used in the xpp framework and conversion functions
(`ros_helpers.h`) between these messages and some xpp data types.

------------------------------------------------------------------------------------

### Dependencies
- This package depends on https://bitbucket.org/adrlab/xpp_controller, as this is
  where some basic types of xpp are defined.

### Sample trajectories ###
- Saved are some trajectory messages (`\bags`) for hyq that perform walking motions that were 
  generated using https://bitbucket.org/adrlab/xpp_opt. To play back these messages
  to e.g. test your tracking controller, execute:

          $ rosbag play --delay 1 8_steps_forward.bag

- An implementation of a controller that reads in these messages and executes them
  can be found in https://bitbucket.org/adrlab/xpp_walking_controller.

### Who do I talk to?
- Alexander Winkler (winklera@ethz.ch)