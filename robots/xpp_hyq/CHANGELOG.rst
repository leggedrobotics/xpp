^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package xpp_vis_hyq
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.10 (2019-04-05)
-------------------
* add xacro dependency
* Contributors: Alexander Winkler

1.0.9 (2018-07-10)
------------------
* Merge pull request #7 from zlingkang/xacropi2xacro
  change deprecated xacro.py to xacro
* Contributors: zlingkang

1.0.8 (2018-07-07)
------------------
* avoid IK segfault for biped/hyq when not enough ee positions sent
* removed limits on hyqleg joint angles
* Contributors: Alexander Winkler

1.0.7 (2018-07-03)
------------------
* add rviz launch specifically for hyq
* Contributors: Alexander Winkler

1.0.6 (2018-04-18)
------------------
* add missing libxpp_hyq.so library to catkin install
* Contributors: Alexander Winkler

1.0.5 (2018-02-01)
------------------
* modernize CMake files
* use default keyword for empty destructors
* use default BSD license
* Contributors: Alexander Winkler

1.0.4 (2018-01-03)
------------------
* Merge pull request #3 from leggedrobotics/devel
  Fix xacro bug and remove terrain and optimization specific classes.
* xpp_hyq: add --inorder when processing xacro files
  see http://wiki.ros.org/xacro#Processing_Order
* cleaned up formatting of xacro files
* removed unused xacro origin tag causing run failure
* Contributors: Alexander W Winkler

1.0.3 (2017-11-03)
------------------
* removed xpp_ros_conversions (now in xpp_states)
* 1.0.2
* update changelog
* Contributors: Alexander Winkler

1.0.2 (2017-10-28)
------------------

1.0.1 (2017-10-27)
------------------

1.0.0 (2017-10-26)
------------------
* added separate xpp_example package and Readme.md
* included install space in CMakeLists.txt
* explicitly listing ros dependencies (not through catkin_package macro)
* Added license to files and giving credit to contributors.
* cleaned up package.xml and CMakeLists.txt files
* cleaned up endeffector and joint representation
* showed close connection of mono/bi/quad to hyq
* added doxygen comments to header files
* made xpp_vis robot independent. Added sample hopper executables
* Contributors: Alexander Winkler
