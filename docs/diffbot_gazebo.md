## Simulate DiffBot in Gazebo

As described in the [Creating your own Gazebo ROS Package](http://gazebosim.org/tutorials?tut=ros_roslaunch#CreatingyourownGazeboROSPackage), it is common in ROS to create a package that contains all the world files and launch files used with Gazebo. These files are located in a ROS package named `/MYROBOT_gazebo`. For DiffBot the package is named `diffbot_gazebo`. Another example that follows best pratices is `rrbot` which can be found in the [gazebo_ros_demos](https://github.com/ros-simulation/gazebo_ros_demos) repository.

```console
fjp@ubuntu:~/git/diffbot/ros/src$ catkin create pkg diffbot_gazebo
Creating package "diffbot_gazebo" in "/home/fjp/git/diffbot/ros/src"...
Created file diffbot_gazebo/package.xml
Created file diffbot_gazebo/CMakeLists.txt
Successfully created package files in /home/fjp/git/diffbot/ros/src/diffbot_gazebo.
```
 
The `diffbot_gazebo` package contains a launch file to lauch a world in Gazebo and spawn the robot model, 
which is defined in the previously created `diffbot_description` package. 
For the launch files the convention is to have a folder named `launch` and for Gazebo world files a folder named `world` inside a package.
 
 ```console
 fjp@ubuntu:~/git/diffbot/ros/src/diffbot_gazebo$ mkdir launch world
 ```
 
 Inside the `launch` folder is the `diffbot.launch`.
 
 ```xml
<launch>
    <!-- these are the arguments you can pass this launch file, for example paused:=true -->
    <arg name="paused" default="false"/>
    <arg name="use_sim_time" default="true"/>
    <arg name="gui" default="true"/>
    <arg name="headless" default="false"/>
    <arg name="debug" default="false"/>

    <!-- We resume the logic in empty_world.launch, changing only the name of the world to be launched -->
    <include file="$(find gazebo_ros)/launch/empty_world.launch">
        <arg name="world_name" value="$(find diffbot_gazebo)/worlds/diffbot.world"/>
        <arg name="debug" value="$(arg debug)" />
        <arg name="gui" value="$(arg gui)" />
        <arg name="paused" value="$(arg paused)"/>
        <arg name="use_sim_time" value="$(arg use_sim_time)"/>
        <arg name="headless" value="$(arg headless)"/>
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

Then its possible to launch the `diffbot.launch` with:

```console
roslaunch diffbot_gazebo diffbot.launch
```

This will lead to the following output:

```console
roslaunch diffbot_gazebo diffbot.launch 
... logging to /home/fjp/.ros/log/6be90ef2-fdd8-11ea-9cb3-317fd602d5f2/roslaunch-tensorbook-393333.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://tensorbook:32837/

SUMMARY
========

PARAMETERS
 * /gazebo/enable_ros_network: True
 * /rosdistro: noetic
 * /rosversion: 1.15.8
 * /use_sim_time: True

NODES
  /
    gazebo (gazebo_ros/gzserver)
    gazebo_gui (gazebo_ros/gzclient)

ROS_MASTER_URI=http://localhost:11311

process[gazebo-1]: started with pid [393352]
process[gazebo_gui-2]: started with pid [393357]
[ INFO] [1600950165.494721382]: Finished loading Gazebo ROS API Plugin.
[ INFO] [1600950165.495515766]: waitForService: Service [/gazebo/set_physics_properties] has not been advertised, waiting...
[ INFO] [1600950165.649461740]: Finished loading Gazebo ROS API Plugin.
[ INFO] [1600950165.650277038]: waitForService: Service [/gazebo_gui/set_physics_properties] has not been advertised, waiting...
[ INFO] [1600950166.640929113]: waitForService: Service [/gazebo/set_physics_properties] is now available.
[ INFO] [1600950166.659917502, 0.007000000]: Physics dynamic reconfigure ready.
```

Also, the Gazebo simulator will open a new window with the objects defined in `diffbot.world` except for the Gas station because it is a model 
that has to be downloaded first, which is happening in the background. TODO: gas station is not showing this way.


<figure>
    <a href="/assets/collections/diffbot/gazebo/empty-world.png"><img src="/assets/collections/diffbot/gazebo/empty-world.png"></a>
    <figcaption>Empty world of DiffBot.</figcaption>
</figure>


To get the Gas station or other available models it is possible to clone the [gazebo models repository](https://github.com/osrf/gazebo_models) into your `/home/your_username/.gazebo` folder, e.g.:

```console
/home/fjp/.gazeb$ git clone osrf/gazebo_models
```
Then add this path inside Gazebo to insert these models into your world file.

 
 ### Using ROS launch to Spawn URDF Robots
 
 According to the [Gazebo roslaunch tutorial](http://gazebosim.org/tutorials?tut=ros_roslaunch#UsingroslaunchtoSpawnURDFRobots) the recommended way
 to spawn a robot into Gazebo is to use a launch file. Therefore, edit the `diffbot.launch` inside the `diffbot_gazebo` package by adding the following inside the `<launch> </launch` tag:
 
 ```console
    <!-- Load the URDF into the ROS Parameter Server -->
    <param name="robot_description"
        command="$(find xacro)/xacro --inorder '$(find diffbot_description)/urdf/diffbot.xacro'" />

    <!-- Run a python script to the send a service call to gazebo_ros to spawn a URDF robot -->
    <node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model" respawn="false" output="screen"
        args="-urdf -model diffbot -param robot_description"/>
 ```

See also the complete [`diffbot.launch`](https://github.com/fjp/diffbot/blob/master/ros/src/diffbot_gazebo/launch/diffbot.launch) file.


This will open Gazebo simulator and show the DiffBot model:

<figure>
    <a href="/assets/collections/diffbot/gazebo/diffbot_gazebo.png"><img src="/assets/collections/diffbot/gazebo/diffbot_gazebo.png"></a>
    <figcaption>Empty world including DiffBot.</figcaption>
</figure>
