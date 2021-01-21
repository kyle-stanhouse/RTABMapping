# 'Map My World' project

THE CONTENT BELOW IS NOT APPLICABLE TO MAP MY WORLD PROJECT

## Summary

Create a 2D occupancy grid and 3D octomap from a simulated environment using my own robot with the RTAB-Map package (rtabmap_ros).


## Directory Structure

```
    .RTABMapping                      # WhereAmI Project
    ├── README.md                       # my_robot package 
    ├── my_robot                       # my_robot package                   
    │   ├── launch                     # launch folder for launch files   
    │   │   ├── robot_description.launch
    │   │   ├── world.launch
    │   │   ├── mapping.launch
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
    │   │   ├── rtabmap.rviz
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
* '$ sudo apt-get install ros-kinetic-rtabmap-ros\

Perhaps update your system\
`sudo apt-get update && sudo apt-get upgrade -y`\

## Install and run 

* Create a `catkin_ws`\
`$ mkdir -p /home/workspace/catkin_ws/src`\
`$ cd /home/workspace/catkin_ws/src`\
`$ catkin_init_workspace`

* Clone RTABMapping Repo\
`$ git clone https://github.com/kyle-stanhouse/RTABMapping`
(Optional) Teleop pkg
`cd /home/workspace/catkin_ws/src`\
`git clone https://github.com/ros-teleop/teleop_twist_keyboard\

* Build application\
`$ cd /home/workspace/catkin_ws`\
`$ catkin_make`

* Run project\
Open up terminal 1\
`$ roslaunch my_robot world.launch`\
(Optional) Open up terminal 2\
`$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py`\
Open up terminal 3\
`$ roslaunch my_robot mapping.launch`

## Testing
  
* Navigate the robot in the simulation to create a map for the environment. When you are all set, terminal the node and you could find your map db file in the place you specified in the launch file. If you did not modify the argument, it will be located in the /root/.ros/ folder. \   

* Note: You will notice that currently my simulation does not execute Nav 2D commands well, and therefore it is better to use the tele-op option.\

* The project rubric states that 3 loop closures will be sufficient for mapping the entire environment. You can maximize your loop closures by going over similar paths two or three times. This allows for the maximization of feature detection, facilitating faster loop closures! When you are done mapping, be sure to copy or move your database before moving on to map a new environment. Remember, relaunching the mapping node deletes any database in place on launch start up!




