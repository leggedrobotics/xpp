# README #

### What is this repository for? ###

* This repository creates a ros node that takes in a state trajectory message for HyQ and sends it out to rviz for visualization. All you have to do for this is publish a message under the topic "/hyq_rviz_trajectory" 
and the `xpp_vis node` of this repo will convert and forward it to rviz to be displayed.

### How do I get set up? ###

* Clone this project into your catkin workspace/src folder
* Do the same with the dependecy: `git clone git@bitbucket.org:adrlab/hyqb_description.git`
* Install eigen: `sudo apt-get install libeigen3-dev`
* Compile your ros workspace: `catkin_make` or `catkin build`
* Run the launch script `roslaunch xpp_vis xpp_vis_example.launch` to run an example
* Launch `rviz` and `Add->RobotModel` to display the sent marker message
* In Rviz, change the "Robot Description" ros parameter to "/hyq_rviz_urdf_robot_description"
* Set the `FixedFrame` in rviz (top left corner) to "world"


### Who do I talk to? ###

* Alexander Winkler (winklera@ethz.ch)