^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package xpp_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.10 (2019-04-05)
-------------------
* Improve docs (`#8 <https://github.com/leggedrobotics/xpp/issues/8>`_)
* Contributors: Alexander Winkler

1.0.9 (2018-07-10)
------------------

1.0.8 (2018-07-07)
------------------

1.0.7 (2018-07-03)
------------------

1.0.6 (2018-04-18)
------------------

1.0.5 (2018-02-01)
------------------
* modernize CMake files
* use default BSD license
* Contributors: Alexander Winkler

1.0.4 (2018-01-03)
------------------
* Merge pull request #3 from leggedrobotics/devel
  Fix xacro bug and remove terrain and optimization specific classes.
* removed optimization related topic names
* rename OptParameters.msg to more concise RobotParameters.msg
* removed UserCommand msg and terrain_builder class
* renamed some launch files
* Contributors: Alexander W Winkler

1.0.3 (2017-11-03)
------------------
* 1.0.2
* update changelog
* Contributors: Alexander Winkler

1.0.2 (2017-10-28)
------------------

1.0.1 (2017-10-27)
------------------

1.0.0 (2017-10-26)
------------------
* removed robot specific bag files from xpp_msgs
* added separate xpp_example package and Readme.md
* included install space in CMakeLists.txt
* changed catkin pkg dependencies to roscpp
* formatting
* Added license to files and giving credit to contributors.
* cleaned up package.xml and CMakeLists.txt files
* removed biped and quadruped joint identifier
* Merge remote-tracking branch 'xpp_msgs/master'
  Conflicts:
  .gitignore
* Contributors: Alexander Winkler
