## DiffBot Navigation Package

```console
fjp@diffbot:~/catkin_ws/src/diffbot$ catkin create pkg diffbot_navigation --catkin-deps amcl map_server move_base diffbot_bringup                           
Creating package "diffbot_navigation" in "/home/fjp/git/ros_ws/src/diffbot"...
Created file diffbot_navigation/package.xml
Created file diffbot_navigation/CMakeLists.txt
Successfully created package files in /home/fjp/git/ros_ws/src/diffbot/diffbot_navigation.
```

After this we create the required launch files and parameter configurations. These will be used for the simulation and the real robot.
First we focus on the simulation in Gazebo.

### Launch files

All launch files are in the folder named `launch` of the `diffbot_navigation` package.


### Parameter Configuration

The parameters for the navigation package go into the `param` folder.

- [Setup and Configuration of the Navigation Stack on a Robot](http://wiki.ros.org/navigation/Tutorials/RobotSetup)

- [`amcl`](http://wiki.ros.org/amcl?distro=noetic): amcl is a probabilistic localization system for a robot moving in 2D. 
It implements the adaptive (or KLD-sampling) Monte Carlo localization approach (as described by Dieter Fox), 
which uses a particle filter to track the pose of a robot against a known map.

- [`map_server`](http://wiki.ros.org/map_server?distro=noetic): provides the `map_server` ROS Node, which offers map data as a ROS Service. 
It also provides the `map_saver` command-line utility, which allows dynamically generated maps to be saved to file.

- [`move_base`](http://wiki.ros.org/move_base?distro=noetic): The `move_base` package provides an implementation of an action 
(see the [`actionlib`](http://www.ros.org/wiki/actionlib) package) that, given a goal in the world, will attempt to reach it with a mobile base. 
The `move_base` node links together a global and local planner to accomplish its global navigation task. 
It supports any global planner adhering to the `nav_core::BaseGlobalPlanner` interface specified in the [`nav_core`](http://www.ros.org/wiki/nav_core) package 
and any local planner adhering to the `nav_core::BaseLocalPlanner` interface specified in the `nav_core` package. 
The `move_base` node also maintains two costmaps, one for the global planner, and one for a local planner (see the [`costmap_2d`](http://www.ros.org/wiki/costmap_2d) package) 
that are used to accomplish navigation tasks.

- [`gmapping`](http://wiki.ros.org/gmapping): This package contains a ROS wrapper for OpenSlam's Gmapping. 
The gmapping package provides laser-based SLAM (Simultaneous Localization and Mapping), as a ROS node called slam_gmapping. 
Using slam_gmapping, you can create a 2-D occupancy grid map (like a building floorplan) from laser and pose data collected by a mobile robot.

- [ROS cartographer](https://github.com/cartographer-project/cartographer_ros)
- [`slam_toolbox`](https://github.com/SteveMacenski/slam_toolbox)

Examples
- [TurtleBot3 Navigation](https://github.com/ROBOTIS-GIT/turtlebot3/tree/master/turtlebot3_navigation)
