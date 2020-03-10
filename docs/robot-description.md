### 2WD Robot Description

The description of the 2WD robot will be created in its own package. The description uses URDF and xacro. 
The `2wd_robot_description` package can be with [`catkin create pkg PKG_NAME [--catkin-deps [DEP [DEP ...]]]`](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_create.html#catkin-create-pkg):

```bash
fjp@ubuntu:~/git/2wd-robot/ros/src$ catkin create pkg robot_description --catkin-deps roscpp tf geometry_msgs urdf rviz xacro
Creating package "lm393_speed_sensor" in "/home/fjp/git/2wd-robot/ros/src"...
Created file lm393_speed_sensor/CMakeLists.txt
Created file lm393_speed_sensor/package.xml
Created folder lm393_speed_sensor/include/lm393_speed_sensor
Created folder lm393_speed_sensor/src
Successfully created package files in /home/fjp/git/2wd-robot/ros/src/lm393_speed_sensor.
```

The package depends on the ROS [client library](http://wiki.ros.org/Client%20Libraries) [`roscpp`](http://wiki.ros.org/roscpp).
