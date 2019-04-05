^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package xpp_vis
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.10 (2019-04-05)
-------------------
* checking sum of the contact forces was hiding negative forces. Use norm instead (`#10 <https://github.com/leggedrobotics/xpp/issues/10>`_)
* Improve docs (`#8 <https://github.com/leggedrobotics/xpp/issues/8>`_)
* Contributors: Alexander Winkler, Ruben Grandia

1.0.9 (2018-07-10)
------------------

1.0.8 (2018-07-07)
------------------
* update catkin syntax for unit tests
* Fix unit test through visualizer improvement.
* fix wrongly displaying old markers when switching robots.
* make printouts in DEVEL only
* Contributors: Alexander Winkler

1.0.7 (2018-07-03)
------------------
* modified default rviz launch scripts
* Contributors: Alexander Winkler

1.0.6 (2018-04-18)
------------------

1.0.5 (2018-02-01)
------------------
* update launch scripts
* cleaned-up some cmake files
* modernize CMake files
* move terrain types to towr_ros
* Contributors: Alexander Winkler

1.0.4 (2018-01-03)
------------------
* Merge pull request #3 from leggedrobotics/devel
  Fix xacro bug and remove terrain and optimization specific classes.
* removed optimization related topic names
* rename OptParameters.msg to more concise RobotParameters.msg
* removed UserCommand msg and terrain_builder class
* use =default for empty c'tor/d'tor.
* [smell] removed quiet return function that gives no warning.
* Contributors: Alexander W Winkler

1.0.3 (2017-11-03)
------------------
* changed rviz default config name
* removed xpp_ros_conversions (now in xpp_states)
* 1.0.2
* update changelog
* Merge pull request #1 from mikaelarguedas/missing_includes
  add missing std includes
* add missing std includes
* Contributors: Alexander W Winkler, Mikael Arguedas

1.0.2 (2017-10-28)
------------------
* Merge pull request `#1 <https://github.com/leggedrobotics/xpp/issues/1>`_ from mikaelarguedas/missing_includes
  add missing std includes
* add missing std includes
* Contributors: Alexander W Winkler, Mikael Arguedas

1.0.1 (2017-10-27)
------------------
* xpp_vis: add visualization_msg dependency
* Contributors: Alexander W Winkler

1.0.0 (2017-10-26)
------------------
* removed robot specific bag files from xpp_msgs
* added separate xpp_example package and Readme.md
* fixed unit test
* included install space in CMakeLists.txt
* explicitly listing ros dependencies (not through catkin_package macro)
* removed user interface node and deprecated keyboard dependency.
* changed catkin pkg dependencies to roscpp
* Added license to files and giving credit to contributors.
* removed some unneccessary dependencies.
* cleaned up package.xml and CMakeLists.txt files
* cleaned up endeffector and joint representation
* showed close connection of mono/bi/quad to hyq
* added doxygen comments to header files
* made xpp_vis robot independent. Added sample hopper executables
* removed biped and quadruped joint identifier
* moved includes into subfolder xpp_vis
* added xpp_vis
* Contributors: Alexander W Winkler
