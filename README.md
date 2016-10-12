# README #

### What is this repository for? ###

* This repository created a ros node that takes in a state trajectory message for HyQ and sends it out to rviz for visualization.

### How do I get set up? ###

* Clone this project into your catkin workspace/src folder
* Do the same with the dependecy: `git clone git@bitbucket.org:adrlab/hyqb_essentials.git`
* Install eigen: `sudo apt-get install libeigen3-dev`
* Compile your ros workspace: `catkin_make` or `catkin build`
* Run the launch script `roslaunch hyqb_visualizer_low_dep hyqb_vis.launch` to run an example
* Launch `rviz` and `Add->RobotModel` to display the sent marker message
* Change the "Robot Description" parameter to "/robot_description"
* Set the `FixedFrame` in rviz (top left corner) to "world"

### Who do I talk to? ###

* Alexander Winkler (winklera@ethz.ch)