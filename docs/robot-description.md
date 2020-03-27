### DiffBot Robot Description

The description of the 2WD robot will be created in its own package named `diffbot_description`. 
The description uses [URDF](https://wiki.ros.org/urdf) and [xacro](https://wiki.ros.org/xacro). 
For this we create we first create a package with [`catkin create pkg PKG_NAME [--catkin-deps [DEP [DEP ...]]]`](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_create.html#catkin-create-pkg):

```bash
fjp@ubuntu:~/git/2wd-robot/ros/src$ catkin create pkg diffbot_description
Creating package "diffbot_description" in "/home/fjp/git/2wd-robot/ros/src"...
Created file diffbot_description/CMakeLists.txt
Created file diffbot_description/package.xml
Successfully created package files in /home/fjp/git/2wd-robot/ros/src/diffbot_description.
```

Because this package contains only descriptions and launch files it doesn't require any dependencies. 

According to ROS conventions we create the following folders where the individual files realted to the robot description will be placed:

```bash
fjp@ubuntu:~/git/2wd-robot/ros/src/robot_description$ mkdir urdf meshes launch
``` 

The `urdf` folder will be used to keep the `urdf` and `xacro` files. 
The `meshes` folder keeps the `meshes` that are included in the `urdf` file, and the `launch` folder keeps the ROS launch files.


### Required Tools

To check a [urdf](https://wiki.ros.org/urdf) file we can make use of the tools `check_urdf` and `urdf_to_graphiz` in the `liburdfdom-tools` debian package. 
Install it with the following command:

```bash
sudo apt install liburdfdom-tools
```

Then it is possible to check urdf files for errors:

```bash
fjp@ubuntu:~/git/2wd-robot/ros/src/robot_description/urdf$ check_urdf robot.urdf
```

And output a graphviz diagram of the robot model:

```bash
fjp@ubuntu:~/git/2wd-robot/ros/src/robot_description/urdf$ urdf_to_graphiz robot.urdf
```

To visualize the 3D model in RVIZ we first need to install the `joint-state-publisher-gui` which was separated from non-gui `joint-state-publisher`. There exists a debian package which can be installed with the following command:

```bash
fjp@ubuntu:~/git/2wd-robot/ros$ sudo apt install ros-melodic-joint-state-publisher-gui
```

### Robot Model

To model the two wheeled differential drive robot we follow [REP-120](https://www.ros.org/reps/rep-0120.html#base-link).
It states to use a `base_link` and a `base_footprint`. 
