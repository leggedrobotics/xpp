^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package xpp_vis
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.2 (2017-10-28)
------------------
* Merge pull request `#1 <https://github.com/leggedrobotics/xpp/issues/1>`_ from mikaelarguedas/missing_includes
  add missing std includes
* add missing std includes
* Contributors: Alexander W Winkler, Mikael Arguedas

1.0.1 (2017-10-27)
------------------
* xpp_vis: add visualization_msg dependency
* Contributors: Alexander Winkler

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
* Contributors: Alexander Winkler
