^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package xpp_ros_conversions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* included install space in CMakeLists.txt
* explicitly listing ros dependencies (not through catkin_package macro)
* removed user interface node and deprecated keyboard dependency.
* removed find_package in ros conversions, as only contains headers
* Added license to files and giving credit to contributors.
* cleaned up package.xml and CMakeLists.txt files
* cleaned up endeffector and joint representation
* showed close connection of mono/bi/quad to hyq
* added doxygen comments to header files
* moved includes into subfolder xpp_vis
* added xpp_ros_conversions pkg
* Contributors: Alexander Winkler
