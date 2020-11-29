## DiffBot Navigation Package

<figure>
    <a href="https://github.com/fjp/diffbot/raw/master/docs/resources/navigation/navigation_stack.png"><img src="https://github.com/fjp/diffbot/raw/master/docs/resources/navigation/navigation_stack.png"></a>
    <figcaption>Navigation Stack Overview.</figcaption>
</figure>

```console
fjp@diffbot:~/catkin_ws/src/diffbot$ catkin create pkg diffbot_navigation --catkin-deps amcl map_server move_base diffbot_bringup                           
Creating package "diffbot_navigation" in "/home/fjp/git/ros_ws/src/diffbot"...
Created file diffbot_navigation/package.xml
Created file diffbot_navigation/CMakeLists.txt
Successfully created package files in /home/fjp/git/ros_ws/src/diffbot/diffbot_navigation.
```

We also need the following ROS packages that can be installed from the ROS Ubuntu packages:

```console
$ sudo apt install ros-noetic-dwa-local-planner ros-noetic-amcl ros-noetic-map-server ros-noetic-move-base
``` 

After this we create the required launch files and parameter configurations. These will be used for the simulation and the real robot.
First we focus on the simulation in Gazebo.

### Launch files

All launch files are in the folder named `launch` of the `diffbot_navigation` package.

Inside the `move_base.launch` it is important to remap the following topics:

```xml
  <!-- Arguments -->
  <arg name="cmd_vel_topic" default="/diffbot/mobile_base_controller/cmd_vel" />
  <arg name="odom_topic" default="/diffbot/mobile_base_controller/odom" />
...
    <!-- remappings of move_base node -->
    <remap from="cmd_vel" to="$(arg cmd_vel_topic)"/>
    <remap from="odom" to="$(arg odom_topic)"/>
``` 

### Parameter Configuration

The parameters for the navigation package go into the `config` (for some robots named `param`) folder.
Most of them can be changed during runtime using [dynamic reconfigure](http://wiki.ros.org/dynamic_reconfigure) with the [`rqt_reconfigure` gui](http://wiki.ros.org/rqt_reconfigure).

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

### Navigation in Gazebo with available Map

To navigate the robot in the simulation run the following command but make sure to first download the 
[turtlebot3_world](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/master/turtlebot3_gazebo/models/turtlebot3_world)  
to your `~/.gazebo/models/` folder. This is required because the `turtlebot3_world.world` file references the `turtlebot3_world` model.


```console
roslaunch diffbot_navigation diffbot.launch world_name:='$(find diffbot_gazebo)/worlds/turtlebot3_world.world'
```

This will spawn DiffBot inside the turtlebot3 world inside Gazebo and visualize the elements of the navigation stack in RViz.

<figure>
    <a href="https://youtu.be/2SwFTrJ1Ofg"><img src="https://github.com/fjp/diffbot/blob/master/docs/resources/navigation/diffbot-navigation-gazebo-turtlebot3-world-small.gif"></a>
    <figcaption>Navigation demo of DiffBot (click to view the demo on Youtube).</figcaption>
</figure>

To navigate the robot using the default [DWA planner](http://wiki.ros.org/dwa_local_planner) in the known map, 
coming from the running [`map_server`](http://wiki.ros.org/map_server), you can use the 
[2D Nav Goal in RViz](http://wiki.ros.org/navigation/Tutorials/Using%20rviz%20with%20the%20Navigation%20Stack#A2D_Nav_Goal).
Just select the navigation arrow to where the robot should move as shown in the animation above.


The DWA local planner is working for differential drive robots, like DiffBot. For other robots such as non-holonomic robots or other types of mobile robots (also differential drive robots) other planners can be used. See for example [`teb_local_planner`](http://wiki.ros.org/teb_local_planner).
{: .notice }


### Resources 

Global Planners:
- [`global_planner`](http://wiki.ros.org/global_planner)

Local Planners:
- [Difference between DWA and Base Local Planner](https://answers.ros.org/question/10718/dwa_planner-vs-base_local_planner/)
- [Difference between DWA and TEB Local Planner](https://answers.ros.org/question/274564/difference-between-dwa-local_planner-and-teb-local_planner/)



