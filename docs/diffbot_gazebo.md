## Simulate DiffBot in Gazebo

As described in the [Creating your own Gazebo ROS Package](http://gazebosim.org/tutorials?tut=ros_roslaunch#CreatingyourownGazeboROSPackage), it is common in ROS to create a package that contains all the world files and launch files used with Gazebo. These files are located in a ROS package named `/MYROBOT_gazebo`. For DiffBot the package is named `diffbot_gazebo`. Another example can be found in the [gazebo_ros_demos](https://github.com/ros-simulation/gazebo_ros_demos) repository.

```console
fjp@ubuntu:~/git/diffbot/ros/src$ catkin create pkg diffbot_gazebo
Creating package "diffbot_gazebo" in "/home/fjp/git/diffbot/ros/src"...
Created file diffbot_gazebo/package.xml
Created file diffbot_gazebo/CMakeLists.txt
Successfully created package files in /home/fjp/git/diffbot/ros/src/diffbot_gazebo.
```
 Replace 'MYROBOT' with the name of your bot in lower case letters.
 
 This package contains a launch file to lauch a world in Gazebo and spawn the robot model defined in the previously created `diffbot_description` package.
 
 ### Using ROS launch to Spawn URDF Robots
 
 According to the [Gazebo roslaunch tutorial](http://gazebosim.org/tutorials?tut=ros_roslaunch#UsingroslaunchtoSpawnURDFRobots) the recommended way
 to spawn a robot into Gazebo is to use a launch file.
