## DiffBot Slam Package

This package contains launch files and configurations for different 
[simultaneous localization and mapping (SLAM)](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping) algorithms
to map the environment of the robot in 2D, although some of these algorithms can be used to map in 3D.

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
The `hector_mapping` is a SLAM approach that can be used without odometry as well as on platforms that exhibit roll/pitch motion (of the sensor, the platform or both), such as drones. It leverages the high update rate of modern LIDAR systems like the Hokuyo UTM-30LX and provides 2D pose estimates at scan rate of the sensors (40Hz for the UTM-30LX). While the system does not provide explicit loop closing ability, it is sufficiently accurate for many real world scenarios. The system has successfully been used on Unmanned Ground Robots, Unmanned Surface Vehicles, Handheld Mapping Devices and logged data from quadrotor UAVs.

Unlike `gmapping` which uses a [particle filter](https://en.wikipedia.org/wiki/Particle_filter), 
`karto`, `cartographer` and `hector_slam` are all [graph-based SLAM algorithms](http://www2.informatik.uni-freiburg.de/~stachnis/pdf/grisetti10titsmag.pdf).
The least accurate SLAM algorithm is `gmapping` but it works fine for smaller maps. Use other algorithms, such as `karto` if you operate your robot in
larger environments or you want more accuracy.

Another interesing package is [`slam_toolbox`](https://github.com/SteveMacenski/slam_toolbox) which provides ROS1 and ROS2 
support and is based on the easy to use `karto` algorithm. `karto` is the basis for many companies because it provides an excellent scan matcher
and can operate in large environments. Additionally, `slam_toolbox` provides tools to edit a generated map and even create a high quality 
map using stored data (offline).


The `cartographer` package is currently supported by OpenRobotics and not by Google where it was originally developed.
It is currently also not setup correctly for DiffBot. Using it will result in errors.
{: .notice }



#### Launch files

This package provides a main launch file named `diffbot_slam.launch` which accepts an argument `slam_method`.
Depending on its value, different launch files will be included that execute the specified SLAM algorithm using its configuration in the `config` folder.

As mentioned above, every ROS slam package requries messages from the laser-range finder topic. Usually this topic is named `/scan`.
To distinguish possible multiple lidars, the topic of DiffBot resides in its namespace `/diffbot/scan`.
Therefore, its necessary to remap the `/scan` topic to `/diffbot/scan`. The following shows how this was done for the `gmapping` launch file.

Inside this package in the [`launch/gmapping.launch`](https://github.com/fjp/diffbot/blob/noetic-devel/diffbot_slam/launch/diffbot_gmapping.launch) 
it is important to map the `scan` topic to laser scanner topic published by Diffbot.
Remappings are done in the [node tag](http://wiki.ros.org/roslaunch/XML/node). Here, for the `gmapping.launch` in the `gmapping` node: 

```xml
<launch>
  <!-- Arguments -->
  <arg name="scan_topic"  default="diffbot/scan"/>
...
  <!-- Gmapping -->
  <node pkg="gmapping" type="slam_gmapping" name="diffbot_slam_gmapping" output="screen">
    ...
    <!-- remapping of gmapping node -->
    <remap from="scan" to="$(arg scan_topic)"/>
  </node>
</launch>
```



#### Parameter Configurations

Most of the configrations are the same as [`turtlebot3_slam/config`](https://github.com/ROBOTIS-GIT/turtlebot3/tree/master/turtlebot3_slam/config).
For detailed description of what each parameter does, please check the individual package documentation of the different SLAM methods. 


### Gazebo Simulation Tests

To test SLAM in the Gazebo simulator run the following two launch files in separate terminals.

First run the simulation with:

```console
roslaunch diffbot_gazebo diffbot.launch world_name:='$(find diffbot_gazebo)/worlds/turtlebot3_world.world'
```

and in a second terminal execute the SLAM algorithm:

```console
roslaunch diffbot_slam diffbot_slam.launch slam_method:=gmapping
```

Here you can choose between different algorithms by changing the value of the `slam_method` argument.
Possible values are `gmapping` (the default), `karto`, `hector` and `cartographer`.

The ROS node graph will look like the following:

<figure>
    <a href="https://raw.githubusercontent.com/fjp/diffbot/noetic-devel/docs/resources/slam/nodegraph-gmapping.svg"><img src="https://raw.githubusercontent.com/fjp/diffbot/noetic-devel/docs/resources/slam/nodegraph-gmapping.svg"></a>
    <figcaption>ROS Node graph after launching Gazebo and gmapping.</figcaption>
</figure>

In the figure we can see that `gmapping` subscribes and publishes to `tf`. 

It requires the transformation from `<the frame attached to incoming scans>` to the `base_link`, which is usually a fixed value, 
broadcast periodically by the `robot_state_publisher`.
Aditionally, it requires the transform from `base_link` to `odom`. This is provided by the odometry system (e.g., the driver for the mobile base).
In the case of DiffBot the odometry system consists of EKF fusion data from the motor encoders and the IMU. 
The provided tf transforms are `map` to `odom` that describes the current estimate of the robot's pose within the map frame.
You can read more about the [required and provided transforms](http://wiki.ros.org/gmapping#Required_tf_Transforms) in the documentation.

### Field Tests

In case you get inaccurate maps follow the official ROS [troubleshooting guide for navigation](http://wiki.ros.org/navigation/Troubleshooting).

### Frontier Exploration

The so far described mapping approaches require manually steering the robot in the unknown environment.
[Frontier exploration](http://www.robotfrontier.com/papers/cira97.pdf) is an approach to move a mobile robot on its own to new frontiers to extend its 
map into new territory until the entire environment has been explored. 

The ROS wiki provides a good [tutorial using Husky robot](http://wiki.ros.org/husky_navigation/Tutorials/Husky%20Frontier%20Exploration%20Demo) how to use the [`frontier_exploration`](http://wiki.ros.org/frontier_exploration) package. A lightweight alternative is the [`explore_lite`](http://wiki.ros.org/explore_lite) package.


### Other SLAM Packages (for 3D Mapping)

- [`hdl_graph_slam`](https://github.com/koide3/hdl_graph_slam): Open source ROS package for real-time 6DOF SLAM using a 3D LIDAR. 
It is based on 3D Graph SLAM with 
[NDT](https://www.researchgate.net/publication/4045903_The_Normal_Distributions_Transform_A_New_Approach_to_Laser_Scan_Matching) 
scan matching-based odometry estimation and loop detection. This method is useful for outdoor.
- [RTAB-Map](http://wiki.ros.org/rtabmap_ros): stands for [Real-Time Appearance-Based Mapping](http://introlab.github.io/rtabmap/) and 
is a RGB-D SLAM approach based on a global loop closure detector with real-time constraints. 
This package can be used to generate a 3D point clouds of the environment and/or to create a 2D occupancy grid map for navigation. 
To do this it requires only a stereo or RGB-D camera for visual odometry. Additional wheel odometry is not required but can improve the result.
- [Loam Velodyne](http://wiki.ros.org/loam_velodyne): Laser Odometry and Mapping (Loam) is a realtime method for state estimation and mapping using a 3D lidar, see also the forked Github repository for [`loam_velodyne`](https://github.com/laboshinl/loam_velodyne). Note that this is not supported officially anymore because it became closed source.
- [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2): Real-Time SLAM for Monocular, Stereo and RGB-D Cameras, 
with Loop Detection and Relocalization Capabilities. See [`orb_slam2_ros`](http://wiki.ros.org/orb_slam2_ros) for the ROS wrapper.
- [slam_toolbox](http://wiki.ros.org/slam_toolbox): This package provides a sped up improved slam karto with updated 
SDK and visualization and modification toolsets. It is a ROS drop in replacement to gmapping, cartographer, karto, hector, etc.
This package supports ROS1 and ROS2 and is suitable for use in commercial products because it can map large environments. And it provides tools
to edit the generated maps. See also the related [ROSCon 2019 video](https://vimeo.com/378682207).


### References

- [`slam_toolbox`](http://wiki.ros.org/slam_toolbox), [Slam Toolbox ROSCon 2019 pdf](https://roscon.ros.org/2019/talks/roscon2019_slamtoolbox.pdf)

Papers:

- [A Tutorial on Graph-Based SLAM](http://www2.informatik.uni-freiburg.de/~stachnis/pdf/grisetti10titsmag.pdf)
- `cartographer` [Real-Time Loop Closure in 2D LIDAR SLAM](https://static.googleusercontent.com/media/research.google.com/de//pubs/archive/45466.pdf)
- `hector_slam` [A flexible and scalable SLAM system with full 3D motion estimation](https://www.researchgate.net/publication/228852006_A_flexible_and_scalable_SLAM_system_with_full_3D_motion_estimation).
- [A practical introduction to to pose graph slam](https://www.sauravag.com/2017/07/an-practical-introduction-to-pose-graph-slam/)
- [The Normal Distributions Transform: A New Approach to Laser Scan Matching](https://www.researchgate.net/publication/4045903_The_Normal_Distributions_Transform_A_New_Approach_to_Laser_Scan_Matching)
- [RTAB-Map as an Open-Source Lidar and Visual SLAM Library for Large-Scale and Long-Term Online Operation](https://introlab.3it.usherbrooke.ca/mediawiki-introlab/images/7/7a/Labbe18JFR_preprint.pdf)
- [LOAM: Lidar Odometry and Mapping in Real-time](https://ri.cmu.edu/pub_files/2014/7/Ji_LidarMapping_RSS2014_v8.pdf)
- [ORB-SLAM2: an Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras](https://arxiv.org/abs/1610.06475)
