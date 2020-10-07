## DiffBot Robot Description

The description of the 2WD robot will be created in its own package named `diffbot_description`. 
The description uses [URDF](https://wiki.ros.org/urdf) and [xacro](https://wiki.ros.org/xacro). 
For this we create a package with [`catkin create pkg PKG_NAME [--catkin-deps [DEP [DEP ...]]]`](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_create.html#catkin-create-pkg):

```console
fjp@ubuntu:~/git/diffbot/ros/src$ catkin create pkg diffbot_description
Creating package "diffbot_description" in "/home/fjp/git/diffbot/ros/src"...
Created file diffbot_description/CMakeLists.txt
Created file diffbot_description/package.xml
Successfully created package files in /home/fjp/git/diffbot/ros/src/diffbot_description.
```

Because this package contains only descriptions and launch files it doesn't require any dependencies. 

According to ROS conventions we create the following folders where the individual files realted to the robot description will be placed:

```console
fjp@ubuntu:~/git/diffbot/ros/src/robot_description$ mkdir urdf meshes launch
``` 

The `urdf` folder will be used to keep the `urdf` and `xacro` files. 
The `meshes` folder keeps the `meshes` that are included in the `urdf` file, and the `launch` folder keeps the ROS launch files.

### Robot Model

To model the two wheeled differential drive robot we follow [REP-120](https://www.ros.org/reps/rep-0120.html#base-link).
It states to use a `base_link` and a `base_footprint`. The resulting description files can be found in the [`diffbot_description`](https://github.com/fjp/diffbot/tree/master/ros/src/diffbot_control) package.


### Required Tools

To check a [urdf](https://wiki.ros.org/urdf) file we can make use of the tools `check_urdf` and `urdf_to_graphiz` in the `liburdfdom-tools` debian package. 
Install it with the following command:

```console
sudo apt install liburdfdom-tools
```

First we need to convert the robot description of DiffBot, which is present as `xacro` file, to a `urdf` file by issuing the following command: 

```console
fjp@ubuntu:~/git/diffbot/ros$ rosrun xacro xacro `rospack find diffbot_description`/urdf/diffbot.urdf.xacro -o /tmp/diffbot.urdf
```

After we've created the `urdf` from the `xacro` file we can check the `urdf` files for errors with:

```console
fjp@ubuntu:/tmp$ check_urdf diffbot.urdf 
robot name is: diffbot
---------- Successfully Parsed XML ---------------
root Link: base_footprint has 1 child(ren)
    child(1):  base_link
        child(1):  caster_link
        child(2):  front_left_wheel
        child(3):  front_right_wheel
```

It is also helpful to output a graphviz diagram of the robot model:

```console
fjp@ubuntu:/tmp$ urdf_to_graphiz diffbot.urdf 
Created file diffbot.gv
Created file diffbot.pdf
fjp@ubuntu:/tmp$ evince diffbot.pdf
```

<figure>
    <a href="https://raw.githubusercontent.com/fjp/diffbot/master/docs/resources/diffbot-tf-tree.png"><img src="https://raw.githubusercontent.com/fjp/diffbot/master/docs/resources/diffbot-tf-tree.png"></a>
    <figcaption>Graphviz diagram of DiffBot URDF robot description.</figcaption>
</figure>


To visualize the 3D model in RViz we first need to install the `joint-state-publisher-gui` which was separated from non-gui `joint-state-publisher`. There exists a debian package which can be installed with the following command:

```console
fjp@ubuntu:~/git/diffbot/ros$ sudo apt install ros-noetic-joint-state-publisher-gui
```

After installing the required dependency, the `view_diffbot.launch` launch file can be executed using [`roslaunch`](http://wiki.ros.org/roslaunch) command:

```console
fjp@ubuntu:~/git/diffbot/ros$ roslaunch diffbot_description view_diffbot.launch
```

According to the launch file's configuration, this will show the robot in RViz together with the `joint-state-publisher-gui` to set the joint values:

<figure>
    <a href="https://raw.githubusercontent.com/fjp/diffbot/master/docs/resources/rviz_diffbot_meshes.png"><img src="https://raw.githubusercontent.com/fjp/diffbot/master/docs/resources/rviz_diffbot_meshes.png"></a>
    <figcaption>DiffBot displayed in RViz.</figcaption>
</figure>

With the robot descripton loaded on the ROS parameter server, it's possible to use the `TF Tree` `rqt` plugin to display the transformation tree (see image above).

In the next section, [Gazebo Simulation](/projects/diffbot/ros-packages/gazebo/), the robot model is prepared for simulation inside of 
[Gazebo](http://gazebosim.org/).
