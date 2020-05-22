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
