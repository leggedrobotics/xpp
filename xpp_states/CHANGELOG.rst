^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package xpp_states
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.10 (2019-04-05)
-------------------
* Improve docs (`#8 <https://github.com/leggedrobotics/xpp/issues/8>`_)
* Contributors: Alexander Winkler

1.0.9 (2018-07-10)
------------------

1.0.8 (2018-07-07)
------------------
* avoid IK segfault for biped/hyq when not enough ee positions sent
* Contributors: Alexander Winkler

1.0.7 (2018-07-03)
------------------
* add names to biped and quadruped ids
* Contributors: Alexander Winkler

1.0.6 (2018-04-18)
------------------

1.0.5 (2018-02-01)
------------------
* modernize CMake files
* use default keyword for empty destructors
* use default BSD license
* move terrain types to towr_ros
* Contributors: Alexander Winkler

1.0.4 (2018-01-03)
------------------

1.0.3 (2017-11-03)
------------------
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

1.0.0 (2017-10-26)
------------------
* removed robot specific bag files from xpp_msgs
* included install space in CMakeLists.txt
* changed catkin pkg dependencies to roscpp
* formatting
* Added license to files and giving credit to contributors.
* removed some unneccessary dependencies.
* cleaned up package.xml and CMakeLists.txt files
* cleaned up endeffector and joint representation
* showed close connection of mono/bi/quad to hyq
* made xpp_vis robot independent. Added sample hopper executables
* removed biped and quadruped joint identifier
* added xpp_states
* Contributors: Alexander Winkler
