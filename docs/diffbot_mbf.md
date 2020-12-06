## DiffBot Move Base Flex

As described in the [`move_base_flex` ROS wiki](http://wiki.ros.org/move_base_flex):

Move Base Flex (MBF) is a backwards-compatible replacement for move_base. MBF can use existing plugins for move_base, and provides an enhanced version of the planner, controller and recovery plugin ROS interfaces. It exposes action servers for planning, controlling and recovering, providing detailed information of the current state and the plugin’s feedback. An external executive logic can use MBF and its actions to perform smart and flexible navigation strategies. Furthermore, MBF enables the use of other map representations, e.g. meshes or grid_map This package is a meta package and refers to the Move Base Flex stack packages.The abstract core of MBF – without any binding to a map representation – is represented by the [`mbf_abstract_nav`](http://wiki.ros.org/mbf_abstract_nav) and the [`mbf_abstract_core`](http://wiki.ros.org/mbf_abstract_core). For navigation on costmaps see 
[`mbf_costmap_nav`](http://wiki.ros.org/mbf_costmap_nav) and 
[`mbf_costmap_core`](http://wiki.ros.org/mbf_costmap_core).


This `diffbot_mbf` package was created using `catkin-tools` using the following command:

```console
catkin create pkg diffbot_mbf                                       
Creating package "diffbot_mbf" in "/home/fjp/git/ros_ws/src/diffbot"...
Created file diffbot_mbf/package.xml
Created file diffbot_mbf/CMakeLists.txt
Successfully created package files in /home/fjp/git/ros_ws/src/diffbot/diffbot_mbf.
```

Additionally the following Ubuntu packages are required dependencies of `move_base_flex`:

```console
sudo apt install ros-noetic-mbf-costmap-nav
```


Another working example for turtlebot3 can be found in the [`turtlebot3_mbf`](https://github.com/Rayman/turtlebot3_mbf) package.