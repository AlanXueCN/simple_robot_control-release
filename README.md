simple_robot_control
====================

Python and C++ libraries for easily controlling the PR2. 

Example code can be found in:

  * src/test_app.cpp  (`rosrun simple_robot_control simple_robot_control_test`)
  * simple_robot_control_example.ipynb (`ipython notebook`)

Both assume that you're running:

  * (PR2): roslaunch /etc/ros/robot.launch
  * (anywhere): roslaunch simple_robot_control simple_robot_control.launch

Notes:

  * The hydro branch fixed inconsistent naming in some of the cpp functions - now "To" is always capitalized. 
  * The hydro branch does not actually implement specifying which planner to use for some of the arm_control functions that expect it.


