# 'Map My World' project

THE CONTENT BELOW IS NOT APPLICABLE TO MAP MY WORLD PROJECT

## Summary

Create a ROS package that launches a custom robot model in a custom Gazebo world.\
Utilize the ROS AMCL package and the Tele-Operation / Navigation Stack to localize the robot.\
Explore, add, and tune specific parameters corresponding to each package to achieve the best possible localization results.

## Directory Structure

```
    .Localization                      # WhereAmI Project
    ├── README.md                       # my_robot package 
    ├── my_robot                       # my_robot package                   
    │   ├── launch                     # launch folder for launch files   
    │   │   ├── robot_description.launch
    │   │   ├── world.launch
    │   │   ├── amcl.launch
    │   ├── config                     # config files for nav  
    │   │   ├── base_local_planner_params.yaml
    │   │   ├── costmap_common_params.yaml
    │   │   ├── global_costmap_params.yaml
    │   │   ├── local_costmap_params.yaml    
    │   ├── meshes                     # meshes folder for sensors (lidar)
    │   │   ├── hokuyo.dae
    │   ├── urdf                       # urdf folder for xacro description files
    │   │   ├── my_robot.gazebo
    │   │   ├── my_robot.xacro
    │   ├── world                      # world folder for world files
    │   │   ├── ball_chaser_world.world
    │   │   ├── ball_chaser_world_map.world # use to generate map 
    │   ├── rviz                      # rviz folder for description files
    │   │   ├── gochaseit.rviz
    │   ├── maps                      # map for localization and yaml file
    │   │   ├── map.pgm
    │   │   ├── map.yaml    
    │   ├── CMakeLists.txt             # compiler instructions
    │   ├── package.xml                # package info
```

## Dependencies

ROS Kinetic\
Gazebo >= 7.0\
Git lfs\
Relevant ROS pkgs\
* `$ sudo apt-get install ros-kinetic-navigation`\
* `$ sudo apt-get install ros-kinetic-map-server`\
* `$ sudo apt-get install ros-kinetic-move-base`\
* `$ sudo apt-get install ros-kinetic-amcl`\

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
* Option 1: Send 2D Navigation Goal\
  Your first option would be sending a 2D Nav Goal from RViz.\
  The move_base will try to navigate your robot based on the localization.\
  Based on the new observation and the odometry, the robot to further perform the localization.

  Click the 2D Nav Goal button in the toolbar, then click and drag on the map to send the goal to the robot.\ 
  It will start moving and localize itself in the process.\
  If you would like to give amcl node a nudge, you could give the robot an initial position estimate on the map using 2D Pose Estimate.

* Note: You will notice that currently my simulation does not execute Nav 2D commands well, and therefore it is better to use the tele-op option.\
        I have submitted the project without this feaure completely working due to the fact that it is not included in the project rubric.

* Option 2: Use tele-op pkg
  You could also use teleop node to control your robot and observe it localize itself in the environment if you have set up the optional tele-op pkg.\
  Open another terminal and launch the teleop script: `$rosrun teleop_twist_keyboard teleop_twist_keyboard.py`

  You should be able to control your robot using keyboard commands. Follow the instruction in the terminal for which keys to use



