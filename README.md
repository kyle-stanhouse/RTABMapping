# Localization, 'WhereAmI' project

## Summary

Create a ROS package that launches a custom robot model in a custom Gazebo world.\
Utilize the ROS AMCL package and the Tele-Operation / Navigation Stack to localize the robot.\
Explore, add, and tune specific parameters corresponding to each package to achieve the best possible localization results.

## Dependencies

ROS Kinetic\
Gazebo >= 7.0\
Git lfs\
Relevant ROS pkgs\
`$ sudo apt-get install ros-kinetic-navigation`\
`$ sudo apt-get install ros-kinetic-map-server`\
`$ sudo apt-get install ros-kinetic-move-base`\
`$ sudo apt-get install ros-kinetic-amcl`\

Perhaps update your system\
`sudo apt-get update && sudo apt-get upgrade -y`\

## Install and run 

* Create a `catkin_ws`\
`$ mkdir -p /home/workspace/catkin_ws/src`\
`$ cd /home/workspace/catkin_ws/src`\
`$ catkin_init_workspace`

* Clone Localization Repo\
`$ git clone https://github.com/kyle-stanhouse/Localization.git`
(Optional) Teleop pkg
`cd /home/workspace/catkin_ws/src`\
`git clone https://github.com/ros-teleop/teleop_twist_keyboard\

* Build application\
`$ cd /home/workspace/catkin_ws`\
`$ catkin_make`

* Run project\
Open up terminal 1\
`$ roslaunch my_robot world.launch`\
Open up terminal 2\
`$ roslaunch my_robot amcl.launch`

## Testing
Option 1: Send 2D Navigation Goal\
Your first option would be sending a 2D Nav Goal from RViz.\
The move_base will try to navigate your robot based on the localization.\
Based on the new observation and the odometry, the robot to further perform the localization.\

Click the 2D Nav Goal button in the toolbar, then click and drag on the map to send the goal to the robot.\ 
It will start moving and localize itself in the process.\
If you would like to give amcl node a nudge, you could give the robot an initial position estimate on the map using 2D Pose Estimate.\

Option 2: Use teleop Node
You could also use teleop node to control your robot and observe it localize itself in the environment,\
if you have set it up in the Optional: Teleop Package part.\

Open another terminal and launch the teleop script: rosrun teleop_twist_keyboard teleop_twist_keyboard.py\

You could control your robot by keyboard commands now.\

