# manipulators-for-learning
Simulation source code and examples for applying machine learning on Robotic Manipulators.

## Dependencies:
* Ubuntu 16.04
* [http://wiki.ros.org/kinetic/Installation/Ubuntu](ros-kinetic)
* moveit : `sudo apt-get install ros-kinetic-moveit`
* ur5_moveit_config : `sudo apt-get install ros-kinetic-ur5-moveit-config`
* Python 2.7

Note : If you are using any other ros-<distro> change the `kinetic` to your <distro>.

## Install the package:
* `git clone https://github.com/ioarun/manipulators-for-learning.git`
* `mv manipulators-for-learning/ur5 /your/catkin_ws/src`
* `cd catkin_ws/`
* `catkin_make`
* `. /devel/setup.bash`

## Launch the simulation:
* `roslaunch ur5_gazebo ur5_cubes.launch`
* `roslaunch ur5_gazebo ur5_moveit_planning_execution.launch sim:=true limited:=true`

## Control Script:
* `python catkin_ws/src/ur5/ur5_gazebo/scripts/ur5_gazebo_ctrl.py`


