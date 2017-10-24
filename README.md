### <img src="https://i.imgur.com/erKszlO.png?1" height="100" />

Xpp is a package for the visualization of motion plans for legged robots. Apart from drawing support areas, contact forces and motion trajectories in RVIZ, it also displays these plans for specific robots.  Current robots include a one-legged, a two-legged hopper and [HyQ].

| ![](https://i.imgur.com/NkL8Haw.gif) | ![](https://i.imgur.com/RrEc2Cd.gif) 
|:-------------------------:|:-------------------------:|
|||

The source code is released under a [BSD 3-Clause license](LICENSE).

**Author/Maintainer: [Alexander W. Winkler](https://awinkler.github.io/)**   
**Affiliation: [Robotics Systems Lab, ETH Zurich](http://www.rsl.ethz.ch/)**    

See the [list of contributors](AUTHORS.txt) for further contributors.

[![Build Status](https://ci.leggedrobotics.com/buildStatus/icon?job=github_leggedrobotics/xpp/master)](https://ci.leggedrobotics.com/job/github_leggedrobotics/job/xpp/job/master/)



### ![](data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAACAAAAAgCAMAAABEpIrGAAABR1BMVEUAAAAAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgSVLCGFAAAAbHRSTlMAAQIDBAUGBwgJCgsMDQ4PERIUGBkeICEjJicqLC40NTg7PD0+QEJDREdJTE9QUlVXWVtcXV9jbG9xdHV3e3yCg4WGiIyOkZSXnqKjpqiqq62yt7q+wcPHyMrM09fa3N7k5ujp7/Hz9ff5+/1eW4y7AAABY0lEQVQYGXXB919SUQDG4fcA1xxUYGraMKShZoP2NCmxaJirHFmaoXiB7///c+dw5QPnAs+jSPbR+9fTRn29xTnIqo9PRKqj6ukjLccZ9bCCNW/SO0Dlgrq8wLolKdgH/o4o5iLWnJzBQ+AgkO8hUFAkXQE+yFcEjM5kq0BKnnfAgFoWgDF57gBlOcHj27oOjMsThEBJ1hqUvgNp+XJYRUlHNP1S3H2sRWmapgl1KWAtJlPbWD8lZWYL+WF1eEqH5aCM88qo7TkdakS+GrXda2CFuQU6rBq1DT9YLt4dkJ7g5IfmsdYTijNbOGXpWgPYSChmjsikNFUHvilmC07yp9TTki7XgJx8Vfiswdx5ORPAunx/4CipljU4lu8NsJlUxPyGQ/lGQuBHUk0lYEkxVxrAdkqSKWNlFDdVB3bPKfEFa0bdJmvAyeo/rJvqZTzkzA31dqmCE15VP8GzvdP9l0Ny/gPN9YtNJ8GGPQAAAABJRU5ErkJggg==) Dependencies

[ROS]  
Packages: catkin, roscpp, tf, kdl_parser, robot_state_publisher, message_runtime, message_generation, std_msgs, geometry_msgs, sensor_msgs, rviz, rosbag
      
    sudo apt-get install ros-[ros_distro_name]-[pkg_name]
 
[Eigen]

    sudo apt-get install libeigen3-dev


### ![](data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAACAAAAAgCAMAAABEpIrGAAABBVBMVEUAAAAAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQYVvkvAAAAVnRSTlMAAQIDBAUGBwgJCgsMDQ4PERITGB0gJictNTdJSkxQVVheYmNnaHN0d3h8gIWGiImLjpSXmJudoKKlqKqrrbC6xcfKzNXX2tze4OLk5ujr7fP19/n7/cCQtvYAAAEGSURBVBgZfcGHOsNgGAXg84c2YlQ3Rc1aNWrVKFqKioYunPu/FJHhSfh+7wtBYqlhD+6389Aoj+irQnTAQCsLyTp9TgmiND0fawZkh/RY0FBDeqBj0QedNH0IJE3EWfQZ+KZW3zcQp4b0dBcAzHXIEn6pMXA3Xyf5mUBMtphhzA5iMv0HVBhhjyMq3Sc3ccQfTgpRsz26FrE8ou88iajUGz27RrJ88zJob+UQM/PKQC8HwbTDUBWCqS5DexBM2gztQ2A9M1SDYKLD0DEE5hNDJxCYj+TZypCuU4W/VJukQoVkXUFQpKsw1iQvFCQFBq4UZJf0NBQ0jCZdtwpaRpO8NvCffB5aX0e+Y7YHJ8MRAAAAAElFTkSuQmCC) Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

    cd catkin_workspace/src
    git clone https://github.com/legged_robotics/xpp.git
    cd ..
    catkin_make -DCMAKE_BUILD_TYPE=Release


### ![](data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAACAAAAAgCAMAAABEpIrGAAABKVBMVEUAAAAAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgTOeikIAAAAYnRSTlMAAQMEBQcICQoLDA0OEBESExYXGBkaHh8gISIlJigpLi8xNTg8Q0ZHSUpLTE1QUVVWW11eX2NmaGtscHN0dXl7fIKGkZKYm6C6vsPFx8jKz9HV19rg4ubo6evv8fP19/n7/XGfOiQAAAE1SURBVBgZdcEJNwJRAAXg+5SkQvbsa7IbeylCRNYWIstk7v//EWbem3k6x8z3oVO4RVsOgWboqMGHgGOLkoCtC53i9WoCwDmlGIBN6xh/+lqkuYDkM6WjrlCZ5Ck8sTc6avS0m3ScQBF1BliCJB4YYB5K9IO+ivDM0k9FQGtQMa8Ocvd0LUMbpVIKwzb5TqkCqXt6vVinVIKS+KJ0uToVB1r0mGG4NqilQa0ET4SaAWr70KgZoJaFR1AzQO0OnmFqBo6eLLpScJXpqY7B1pOl1IxD2qVkTfcKKBGL0udaBGLomsoFNNGgDwNagX6sQbgW6e81DOWbAQpQDunP7IfrjLaXlVsqZn6Ptp8UtDx5EwJKlMaBVIvtCXTIZGDboRQFEDkbwH9zlASCJOl4RLD0tm0EnX4Bn0nUATYVlCcAAAAASUVORK5CYII=) Unit Tests

Make sure everything installed correctly by running the unit tests through

    catkin_make run_tests
    
or if you are using [catkin tools].

    catkin build xpp_vis --no-deps --verbose --catkin-make-args run_tests


### ![](data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAACAAAAAgCAMAAABEpIrGAAAAilBMVEUAAAAAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgR83qfXAAAALXRSTlMAAQIDBAcICxITGRobJSYvMD0+S0xcXV5pbG2Cg4WUlaqrusjMztna6PH5+/2o5W36AAAAnklEQVQ4y4XSyQ6CQBAE0AJBXMEddxQXYKT+//c8GSNCdV3nJVPdaWASQOdVrTwJSBYzA5D52ABkFhmA9T7QgKyWngZkkRigtS0bOUcGYL0LNGi2ZVueiQHIfGSAb9tOwHprAN56EpRz+YVb+7LkIZRjXgZyUfepXPWnWwdwGx8KHENAgJ9u/+ARy6MtF4AALvWhwKkPCHAdQiWN5fMbcmNQkFEThW0AAAAASUVORK5CYII=) Usage

A few examples for different robots are provided in the `xpp_examples` package. For starters, run

    roslaunch xpp_examples monoped_ex_bag.launch  // or biped_ex.launch, hyq_ex.launch

These scripts actually executes the following steps:

1. Start the visualization nodes, launch rviz with the correct config and wait for robot messages using `roslaunch xpp_vis xpp_vis.launch`.
 
2. Now robot messages of type `xpp_msgs/RobotStateCartesian.msg` can be sent and visualized. Since we also want to display the URDF of a specific robot, the URDF file of the robot (e.g `monoped_description)/urdf/monoped.urdf` must be uploaded to the ROS parameter server.
  
3. Next a node must be created that transforms `xpp_msgs/RobotStateJoints.msg` into rviz TFs. If we also want to display Cartesian messages `xpp_msgs/RobotStateCartesian.msg`, Inverse Kinematics are neccessary for the specific robots.
 
4. Finally, motions plans must be published for the specific robots. Rosbags of sample motions plans can be found at `xpp_examples/bags/` and can be run using `rosbag play`. To see some examples of how to generate these
bag files or messages, check out `xpp_examples/src/monoped_bag_builder.cc` and `xpp_examples/src/monoped_publisher.cc`.


###  ![](data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAACAAAAAgCAMAAABEpIrGAAABAlBMVEUAAAAAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgQAAgRMgrIEAAAAVXRSTlMAAQIDBAUGBwgJCg0OEBcbHiYoKSorLC0uLzAzNDY6O0BERUpNUV9hY2RmaGxzdHV4f4aMl5qio6iqq62wubq8vsXHyszO0dXX2drg4uTm6+33+fv9Hj8LkAAAARhJREFUOE/FztlawjAUBOBJC0VEcd+34IIbqIhWUBGRTbS4Mu//Kl60tWkTvHWuTjL/lxNAzdLV8ON+V2BMLJckyX5uDKgxiOcY+23+5tIIHiNAWy3s8jwACKVnHgDS54sAgE1yHQCmlVgAMj2++S+0yWVomXgmt/zR6XG0kOyzL+RGeMgMaMhoReHvBhBb2wouPSnrIUiZQAs4/gcwJ6WUngEcSCl3HOSpRgUkSRep9l/gqxDseTAAS/2kawBqj5IOOjGwqoNyDNifGpiNAdFJgmGsR1Vfcar2JbI6UMH3E3kS9fvkNeyLCNSnrDvyKOzXyIYAkCs2G0CxezYDwLolDwPwyn4aWkSNzPrj3s2k3gOiUjFdJ/MDcF6USJu7GQkAAAAASUVORK5CYII=) Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/leggedrobotics/xpp/issues).

[HyQ]: https://www.iit.it/research/lines/dynamic-legged-systems
[ROS]: http://www.ros.org
[rviz]: http://wiki.ros.org/rviz
[catkin tools]: http://catkin-tools.readthedocs.org/
[Eigen]: http://eigen.tuxfamily.org
[Fa2png]: http://fa2png.io/r/font-awesome/link/