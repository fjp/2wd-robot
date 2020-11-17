## DiffBot Slam Package

```console
fjp@diffbot:~/catkin_ws/src/diffbot$ catkin create pkg diffbot_slam --catkin-deps diffbot_navigation gmapping cartographer_ros hector_slam frontier_exploration explore_lite
Creating package "diffbot_slam" in "/home/fjp/git/ros_ws/src/diffbot"...
Created file diffbot_slam/package.xml
Created file diffbot_slam/CMakeLists.txt
Successfully created package files in /home/fjp/git/ros_ws/src/diffbot/diffbot_slam.
```

As you can see this package has lots of dependencies to test different slam implementations and frontier exploration approaches.

### SLAM

SLAM stands for Simultaneous Localization and Mapping sometimes refered to as Concurrent Localization and Mappping (CLAM). The SLAM algorithm combines localization and mapping, where a robot has access only to its own movement and sensory data. The robot must build a map while simultaneously localizing itself relative to the map. See also this [blog post on FastSLAM](https://fjp.at/posts/slam/fastslam/).

#### Launch files


#### Parameter Configurations


### Frontier Exploration

[Frontier exploration](http://www.robotfrontier.com/papers/cira97.pdf) is an approach to move a mobile robot to new frontiers to extend its 
map into new territory until the entire environment has been explored. 

The ROS wiki provides a good [tutorial using Husky robot](http://wiki.ros.org/husky_navigation/Tutorials/Husky%20Frontier%20Exploration%20Demo) how to use the [`frontier_exploration`](http://wiki.ros.org/frontier_exploration) package. A lightweight alternative is the [`explore_lite`](http://wiki.ros.org/explore_lite) package.
