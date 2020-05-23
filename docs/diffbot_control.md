## DiffBot Control Package

As described in the [ROS Integration](https://fjp.at/projects/diffbot/ros-integration/#ros-control) section, 
DiffBot makes use of [ROS Control](https://fjp.at/posts/ros/ros-control/) repositories. 
Specifically the [`diff_drive_controller`](http://wiki.ros.org/diff_drive_controller) package from the 
[`ros_controllers`](https://github.com/ros-controls/ros_controllers) meta package. 
To leverage ROS Control we require our implemented 
[robot description](https://fjp.at/projects/diffbot/ros-packages/robot-description/) and implement 
a class derived from [`hardware_interface::RobotHW`](http://docs.ros.org/melodic/api/hardware_interface/html/c++/classhardware__interface_1_1RobotHW.html).
Let's call it `DiffBotHW` and create it inside a new package named `diffbot_control`, which is created with
[`catkin create pkg PKG_NAME [--catkin-deps [DEP [DEP ...]]]`](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_create.html#catkin-create-pkg):

```console
catkin create pkg diffbot_control --catkin-deps diff_drive_controller hardware_interface controller_manager roscpp transmission_interface sensor_msgs rosparam_shortcuts 
Creating package "diffbot_control" in "/home/fjp/git/diffbot/ros/src"...
Created file diffbot_control/CMakeLists.txt
Created file diffbot_control/package.xml
Created folder diffbot_control/include/diffbot_control
Created folder diffbot_control/src
Successfully created package files in /home/fjp/git/diffbot/ros/src/diffbot_control.
```

To work with this package the specified dependencies must be installed either using the available Ubuntu/Debian packages for ROS melodic or have to be built from source first. The following table lists the dependencies that we have to install because they are not already part of the ROS melodic desktop full installation. Refer to the section [ROS Melodic Setup](https://fjp.at/projects/diffbot/ros-melodic/) for how this was installed. 

| Dependency                    | Source                                                | Ubuntu/Debian Package            |
|:-----------------------------:|:-----------------------------------------------------:|:--------------------------------:|
| `rosparam_shortcuts`          | https://github.com/PickNikRobotics/rosparam_shortcuts | `ros-melodic-rosparam-shortcuts` |
| `hardware_interface`          | https://github.com/ros-controls/ros_control           | `ros-melodic-ros-control`        |
| `transmission_interface`                                                                                                 |
| `diff_drive_controller`       | https://github.com/ros-controls/ros_controllers       | `ros-melodic-ros-controllers`    |

To install a package from source clone (using git) or download the source files from where they are located (commonly hosted on GitHub) into the `src` folder of a ros catkin workspace and execute the [`catkin build`](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html) command. Also make sure to d

```console
cd /homw/fjp/git/diffbot/ros/  # Navigate to the workspace
catkin build              # Build all the packages in the workspace
ls build                  # Show the resulting build space
ls devel                  # Show the resulting devel space
```

Make sure to clone/download the source files suitable for the ROS distribtion you are using. If the sources are not available for the distribution you are working with, it is worth to try building anyway. Chances are that the package you want to use is suitable for multiple ROS distros. For example if a package states in its docs, that it is only available for [kinetic](http://wiki.ros.org/kinetic) it is possible that it will work with a ROS [melodic](http://wiki.ros.org/melodic) install.
{: .notice }
