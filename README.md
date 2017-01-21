# Disclaimer

This is a modified implementation of [Franco Curotto's GraphSLAM implementation using g2o] (https://github.com/francocurotto/GraphSLAM). In this version a robot model modified from [here] (https://github.com/SD-Robot-Vision/PioneerModel) which runs with Gazebo was added along with an animation that works with Gnuplot.

## Requirements

* CMake (https://cmake.org/)
* Eigen3 (http://eigen.tuxfamily.org/index.php?title=Main_Page)
* suitesparse (http://faculty.cse.tamu.edu/davis/suitesparse.html)
* g2o (https://github.com/RainerKuemmerle/g2o)
* Gnuplot (http://www.gnuplot.info/)

This project was developed in linux platform, with C++11 and Python 2.7 

## Modified Files:

- src/python-helpers/*.*
- src/graphSLAM/my_slam.cpp
- src/graphSLAM/slam_functs.cpp

## Compilation

To compile the C++ scripts simply go to the `src/graphSLAM` folder and do:

- `mkdir build`
- `cd build`
- `cmake ..`
- `make`

Notice that you must have g2o installed in your machine.

## Execution

- roslaunch p3dx_gazebo gazebo.launch
- run any feature detector
- rosrun clay_robot stage_controller.py
- ./GraphSLAM/src/python-helpers/gazebo-sim/my-slam.py
