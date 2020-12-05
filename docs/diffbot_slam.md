## DiffBot Slam Package

```console
fjp@diffbot:~/catkin_ws/src/diffbot$ catkin create pkg diffbot_slam --catkin-deps diffbot_navigation gmapping
Creating package "diffbot_slam" in "/home/fjp/git/ros_ws/src/diffbot"...
Created file diffbot_slam/package.xml
Created file diffbot_slam/CMakeLists.txt
Successfully created package files in /home/fjp/git/ros_ws/src/diffbot/diffbot_slam.
```

Additional runtime dependencies are: `cartographer_ros`, `hector_slam`, `frontier_exploration` and `explore_lite`. These
are added to this workspace using [`vcstool`](https://github.com/dirk-thomas/vcstool) (TODO).

As you can see this package has lots of dependencies to test different slam implementations and frontier exploration approaches.
To run this package these dependencies need to be installed and are set as `exec_depend` in the `package.xml`. Currently only `gmapping` provides a ROS Noetic Ubuntu package that can be installed directly with:

```console
sudo apt install ros-noetic-gmapping
```

In case you want to try more advanced SLAM algorithms, such as `karto_slam` or `cartographer_ros` you need the following Ubuntu package dependencies.
Alternatively you can install from source by building the cloned git repository in your catkin workspace.

Take the required installation size into account. For example `karto_slam` needs approximately 125MB because it will also install `ros-noetic-open-karto`.
{: .notice }

```console
sudo apt install ros-noetic-slam-karto
```

### SLAM

SLAM stands for Simultaneous Localization and Mapping sometimes refered to as Concurrent Localization and Mappping (CLAM). The SLAM algorithm combines localization and mapping, where a robot has access only to its own movement and sensory data. The robot must build a map while simultaneously localizing itself relative to the map. See also this [blog post on FastSLAM](https://fjp.at/posts/slam/fastslam/).

To use the following slam algorithms, we need a mobile robot that provides odometry data and is equipped with a horizontally-mounted, 
fixed, laser range-finder. Viewed on a higher level, every specific slam node of these algorithms will attempt to transform each incoming scan into the odom (odometry) [tf](http://wiki.ros.org/tf2) frame. Therefore the node will subscribe to the laser `/scan` and the `/tf` topics. 
Transforms are necessary to relate frames for laser, base, and odometry. The only exception is `hector_slam` which doesn't require odometry for mapping.

The following SLAM implementations are offered using the launch files explained in the next section. It is suggested to start with `gmapping` which is used by default.

- [`gmapping`](http://wiki.ros.org/gmapping): This package contains a ROS wrapper for [OpenSlam's Gmapping](https://openslam-org.github.io/). 
The gmapping package provides laser-based SLAM (Simultaneous Localization and Mapping), as a ROS node called `slam_gmapping`. 
Using `slam_gmapping`, you can create a 2-D occupancy grid map (like a building floorplan) from laser and pose data collected by a mobile robot.
- [`cartographer`](http://wiki.ros.org/cartographer): [Cartographer](https://google-cartographer-ros.readthedocs.io/en/latest/) is a system that provides real-time simultaneous localization and mapping (SLAM) in 2D and 3D across multiple platforms and sensor configurations. See the documentation for an 
[algorithm walkthrough](https://google-cartographer-ros.readthedocs.io/en/latest/algo_walkthrough.html).
- [`karto`](http://wiki.ros.org/karto): This package pulls in the Karto mapping library, and provides a ROS wrapper for using it. Karto is considered more accurate than, for example `gmapping` (note: for ROS noetic, see [`slam_karto`](https://wiki.ros.org/slam_karto)) and became [open source in 2010](https://www.ros.org/news/2010/04/karto-mapping-now-open-source-and-on-coderosorg.html).
- [`hector_slam`](http://wiki.ros.org/hector_slam): metapackage that installs [`hector_mapping`](http://wiki.ros.org/hector_mapping) and related packages. 
The `hector_mapping` is a SLAM approach that can be used without odometry as well as on platforms that exhibit roll/pitch motion (of the sensor, the platform or both). It leverages the high update rate of modern LIDAR systems like the Hokuyo UTM-30LX and provides 2D pose estimates at scan rate of the sensors (40Hz for the UTM-30LX). While the system does not provide explicit loop closing ability, it is sufficiently accurate for many real world scenarios. The system has successfully been used on Unmanned Ground Robots, Unmanned Surface Vehicles, Handheld Mapping Devices and logged data from quadrotor UAVs.

Unlike `gmapping` which uses a [particle filter](https://en.wikipedia.org/wiki/Particle_filter), 
`karto`, `cartographer` and `hector_slam` are all [graph-based SLAM algorithms](http://www2.informatik.uni-freiburg.de/~stachnis/pdf/grisetti10titsmag.pdf).




#### Launch files


Inside the `gmapping` node in the `gmapping.launch` it is important to map the `scan` topic to laser scanner topic published by Diffbot:

```xml
  <!-- Arguments -->
  <arg name="scan_topic"  default="diffbot/scan"/>
...
    <!-- remapping of gmapping node -->
    <remap from="scan" to="$(arg scan_topic)"/>
```

#### Parameter Configurations

### Field Tests

In case you get inaccurate maps follow the official ROS [troubleshooting guide for navigation](http://wiki.ros.org/navigation/Troubleshooting).

### Frontier Exploration

The so far described mapping approaches require manually steering the robot in the unknown environment.
[Frontier exploration](http://www.robotfrontier.com/papers/cira97.pdf) is an approach to move a mobile robot on its own to new frontiers to extend its 
map into new territory until the entire environment has been explored. 

The ROS wiki provides a good [tutorial using Husky robot](http://wiki.ros.org/husky_navigation/Tutorials/Husky%20Frontier%20Exploration%20Demo) how to use the [`frontier_exploration`](http://wiki.ros.org/frontier_exploration) package. A lightweight alternative is the [`explore_lite`](http://wiki.ros.org/explore_lite) package.


### References

- [`slam_toolbox`](http://wiki.ros.org/slam_toolbox), [Slam Toolbox ROSCon 2019 pdf](https://roscon.ros.org/2019/talks/roscon2019_slamtoolbox.pdf)
- [A Tutorial on Graph-Based SLAM](http://www2.informatik.uni-freiburg.de/~stachnis/pdf/grisetti10titsmag.pdf)
