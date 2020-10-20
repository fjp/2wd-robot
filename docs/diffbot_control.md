## DiffBot Control Package

As described in the [ROS Integration](https://fjp.at/projects/diffbot/ros-integration/#ros-control) and 
[Gazebo Simulation](https://fjp.at/projects/diffbot/ros-packages/gazebo/) sections, 
DiffBot makes use of [ROS Control](https://fjp.at/posts/ros/ros-control/) repositories. 
Specifically the [`diff_drive_controller`](http://wiki.ros.org/diff_drive_controller) package from the 
[`ros_controllers`](https://github.com/ros-controls/ros_controllers) meta package. 
To leverage ROS Control for the simulation with Gazebo the [robot description](https://fjp.at/projects/diffbot/ros-packages/robot-description/) and the 
controller configuration (usually a `MYROBOT_control.yaml` file) is required. For the real hardware its required to implement 
a class derived from [`hardware_interface::RobotHW`](http://docs.ros.org/melodic/api/hardware_interface/html/c++/classhardware__interface_1_1RobotHW.html).

The convention to control a robot (in simulation and in the real world) is to have a package named `MYROBOT_control`. In case of DiffBot its called `diffbot_control` and created with
[`catkin create pkg PKG_NAME [--catkin-deps [DEP [DEP ...]]]`](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_create.html#catkin-create-pkg):

```console
catkin create pkg diffbot_control --catkin-deps diff_drive_controller roscpp sensor_msgs 
Creating package "diffbot_control" in "/home/fjp/git/diffbot/ros/src"...
Created file diffbot_control/CMakeLists.txt
Created file diffbot_control/package.xml
Created folder diffbot_control/include/diffbot_control
Created folder diffbot_control/src
Successfully created package files in /home/fjp/git/diffbot/ros/src/diffbot_control.
```

To work with this package the specified dependencies must be installed either using the available Ubuntu/Debian packages for ROS Noetic or have to be built from source first. The following table lists the dependencies that we have to install because they are not already part of the ROS Noetic desktop full installation. Refer to the section [ROS Noetic Setup](https://fjp.at/projects/diffbot/ros-noetic/) for how this was done. 

| Dependency                    | Source                                                | Ubuntu/Debian Package            |
|:-----------------------------:|:-----------------------------------------------------:|:--------------------------------:|
| `diff_drive_controller`       | https://github.com/ros-controls/ros_controllers       | `ros-noetic-ros-controllers`    |

To install a package from source clone (using git) or download the source files from where they are located (commonly hosted on GitHub) into the `src` folder of a ros catkin workspace and execute the [`catkin build`](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html) command. Also make sure to source the workspace after building new packages with `source devel/setup.bash`.

```console
cd /home/fjp/git/diffbot/ros/  # Navigate to the workspace
catkin build              # Build all the packages in the workspace
ls build                  # Show the resulting build space
ls devel                  # Show the resulting devel space
```

Make sure to clone/download the source files suitable for the ROS distribtion you are using. If the sources are not available for the distribution you are working with, it is worth to try building anyway. Chances are that the package you want to use is suitable for multiple ROS distros. For example if a package states in its docs, that it is only available for [kinetic](http://wiki.ros.org/kinetic) it is possible that it will work with a ROS [noetic](http://wiki.ros.org/noetic) install.
{: .notice }


### ROS Control in Gazebo

Two great resources to get the `diff_drive_controller` working inside Gazebo is the [Gazebo ROS Control Tutorial](http://gazebosim.org/tutorials?tut=ros_control)
of [`rrbot`](https://github.com/ros-simulation/gazebo_ros_demos) and the [R2D2 ROS URDF Tutorial](http://wiki.ros.org/urdf/Tutorials/Using%20a%20URDF%20in%20Gazebo), especially the last section, [The Wheels on the Droid Go Round and Round](http://wiki.ros.org/urdf/Tutorials/Using%20a%20URDF%20in%20Gazebo#The_Wheels_on_the_Droid_Go_Round_and_Round).

To spawn DiffBot inside Gazebo, RViz and control it with the [`rqt_robot_steering`](http://wiki.ros.org/rqt_robot_steering) plugin, 
launch the `diffbot.launch` inside the `diffbot_control` package:

```console
roslaunch diffbot_control diffbot.launch
```

This launch file makes use of `diffbot_gazebo/launch/diffbot.launch`, `diffbot_control/launch/diffbot_control.launch` to run gazebo and the `diff_drive_controller`. It also opens RViz with the configuration stored in `diffbot_control/rviz/diffbot.rviz`. 
The following video shows the result of launching. Note the video may be outdated when you read this and the model has improved.

{% include video id="gfhgfU8zUOs" provider="youtube" %}


### ROS Control on the Real Hardware

As mentioned above the its required to implement a class derived from 
[`hardware_interface::RobotHW`](http://docs.ros.org/melodic/api/hardware_interface/html/c++/classhardware__interface_1_1RobotHW.html).
Let's call it `DiffBotHW` and create it inside the `diffbot_control/src` folder.
