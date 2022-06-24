
## LASER-Software
   Software package for LASER (Legged-Agile-Smart-Efficient-Robot) Series

   Details in /laikago_ros
   Software guide and user manual provided by Unitree in /documentation

## Dependencies
  for Ubuntu 16.04
  ROS Kinetic + Gazebo 8
	
  for Ubuntu 18.04
  ROS Melodic + Gazebo 9
  Eigen3 3.3.6 (other vesion can work)

<<<<<<< HEAD
Make these packages have been installed:
```
sudo apt-get install ros-kinetic-controller-manager ros-kinetic-ros-control ros-kinetic-ros-controllers ros-kinetic-joint-state-controller ros-kinetic-effort-controllers ros-kinetic-velocity-controllers ros-kinetic-position-controllers ros-kinetic-robot-controllers ros-kinetic-robot-state-publisher ros-kinetic-gazebo8-ros ros-kinetic-gazebo8-ros-control
```

Change the distro name if using ROS Melodic

## Build Instructions
  * source your ROS
  * build laikago_msgs package first
  `catkin_make --pkg laikago_msgs`
  * complie the packge 
  `catkin_make -DCMAKE_BUILD_TYPE=Release`
  * launch aliengo robot (it can't find the launch file, source ROS workspace again)
  `roslaunch laikago_gazebo aliengo.launch`
  * launch sloped terrain with A1 robot
  `roslaunch laikago_gazebo sloped.launch`
  * or launch A1 robot
  `roslaunch laikago_gazebo a1.launch`
  * run executable file in a new terminal for locomotion
  `rosrun laikago_gazebo laikago_servo`
  The robot will stand up and then start locomotion based on commands from FSM.cpp

  To switch between robots, change `setQuadruped()` function in servo.cpp or anywhere using 	 Quadruped object. Then adjust z-height for QP and MPC in FSM.cpp and ConvexMPCLocomotion.cpp

## Reset simulation
  * run `roscore`
  * open a new terminal and launch the simulation
  * open a new terminal and run executable
  * terminate the executable and run `rosrun laikago_gazebo reset`
  * in the terminal, run `rosservice call /gazebo/reset_simulation "{}"`
  The motion_init() function is called in all of our executables and it can set robot to desired initial condition.

## A1 Jumping
Following the instruction in the build section (If it is your first time to build, remember to make the laikago_msgs first)

* `roslaunch laikago_gazebo a1.launch`
or
* `roslaunch laikago_gazebo a1_onbox.launch`

The A1 robot should be lying on the ground with joints not activated.

* `rosrun laikago_gazebo laikago_servo_jumpMPC`

The A1 robot will jump in ROS simulation.

## Current Status
  * mapping of GRF to joint Torques
  * modify joint controller for torque input
  * stand and change orientation with QP controller
  * added ConvexMPC library
  * plots.py use matplotlib to generate plot from file stream
  * MPC locomotion
  * added more gaits for MPC and switch different gaits
  * posture adjustment on sloped terrain
  * Locomotion with QP controller
  * state estimation from IMU

## Possible Problem during installation
* if the simulation doesn't run well and the time factor is quite low, change the real_time_update_rate smaller based on your computer performance (e.g 100) in /laikago_ros/laikago_gazebo/launch/world/* .world file
 
