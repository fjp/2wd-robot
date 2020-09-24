## Simulate DiffBot in Gazebo

As described in the [Creating your own Gazebo ROS Package](http://gazebosim.org/tutorials?tut=ros_roslaunch#CreatingyourownGazeboROSPackage), it is common in ROS to create a package that contains all the world files and launch files used with Gazebo. These files are located in a ROS package named `/MYROBOT_gazebo`. For DiffBot the package is named `diffbot_gazebo`. Another example can be found in the [gazebo_ros_demos](https://github.com/ros-simulation/gazebo_ros_demos) repository.

```console
fjp@ubuntu:~/git/diffbot/ros/src$ catkin create pkg diffbot_gazebo
Creating package "diffbot_gazebo" in "/home/fjp/git/diffbot/ros/src"...
Created file diffbot_gazebo/package.xml
Created file diffbot_gazebo/CMakeLists.txt
Successfully created package files in /home/fjp/git/diffbot/ros/src/diffbot_gazebo.
```
 
 This package contains a launch file to lauch a world in Gazebo and spawn the robot model defined in the previously created `diffbot_description` package.
 Inside the package, for the launch files the convention is to have a folder named `launch` and for Gazebo world files a folder named `world`.
 
 ```console
 fjp@ubuntu:~/git/diffbot/ros/src/diffbot_gazebo$ mkdir launch world
 ```
 
 Inside the `launch` folder is the `diffbot.launch`.
 
 ```xml
 <launch>
  <!-- We resume the logic in empty_world.launch, changing only the name of the world to be launched -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find MYROBOT_gazebo)/worlds/MYROBOT.world"/>
    <!-- more default parameters can be changed here -->
  </include>
</launch>
```

In the `world` folder of the `diffbot_gazebo` package is the `diffbot.world` file:

```xml
<?xml version="1.0" ?>
<sdf version="1.4">
  <world name="default">
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://gas_station</uri>
      <name>gas_station</name>
      <pose>-2.0 7.0 0 0 0 0</pose>
    </include>
  </world>
</sdf>
```

With these files build the catkin workspace and source it to make the new `diffbot_gazebo` package visible to `roslaunch`:

```console
catkin build
source devel/setup.zsh
```

Then launch the `diffbot.launch` with:

```console
roslaunch diffbot_gazebo diffbot.launch
```


 
 ### Using ROS launch to Spawn URDF Robots
 
 According to the [Gazebo roslaunch tutorial](http://gazebosim.org/tutorials?tut=ros_roslaunch#UsingroslaunchtoSpawnURDFRobots) the recommended way
 to spawn a robot into Gazebo is to use a launch file.
