## DiffBot Base Package

This package contains the so called hardware interface of DiffBot which represents the real hardware in software to work with 
[ROS Control](http://wiki.ros.org/ros_control). 

<figure>
    <a href="https://raw.githubusercontent.com/fjp/diffbot/master/docs/resources/ros_control_overview.png"><img src="https://raw.githubusercontent.com/fjp/diffbot/master/docs/resources/ros_control_overview.png"></a>
    <figcaption><a href="http://wiki.ros.org/ros_control#Overview" title="ROS Control">ROS Control</a> Overview.</figcaption>
</figure>

All that is needed in this package is to write a class that inherits from `hardware_interface::RobotHW` and provide a launch
file. The launch file will 

- Load the robot description from `diffbot_description` to the paramter server
- Run the hardware interface of this package `diffbot_base`
- Load the controller configuration yaml from the `diffbot_control` package to the parameter server
- Load the controllers with the [controller manager](http://wiki.ros.org/controller_manager?distro=noetic)

### diffbot_base Package

The `diffbot_base` package is created with `catkin-tools`:

```console
fjp@diffbot:/home/fjp/git/diffbot/ros/src$ catkin create pkg diffbot_base --catkin-deps diff_drive_controller hardware_interface roscpp sensor_msgs rosparam_shortcuts                 
Creating package "diffbot_base" in "/home/fjp/git/diffbot/ros/src"...
Created file diffbot_base/package.xml
Created file diffbot_base/CMakeLists.txt
Created folder diffbot_base/include/diffbot_base
Created folder diffbot_base/src
Successfully created package files in /home/fjp/git/diffbot/ros/src/diffbot_base.
```

To work with this package the specified dependencies must be installed either using the available Ubuntu/Debian packages for ROS Noetic or have to be built from source first. The following table lists the dependencies that we have to install because they are not already part of the ROS Noetic desktop full installation. Refer to the section [ROS Noetic Setup](https://fjp.at/projects/diffbot/ros-noetic/) for how this was done. 

| Dependency                    | Source                                                | Ubuntu/Debian Package            |
|:-----------------------------:|:-----------------------------------------------------:|:--------------------------------:|
| `rosparam_shortcuts`          | https://github.com/PickNikRobotics/rosparam_shortcuts | `ros-noetic-rosparam-shortcuts` |
| `hardware_interface`          | https://github.com/ros-controls/ros_control           | `ros-noetic-ros-control`        |
| `diff_drive_controller`       | https://github.com/ros-controls/ros_controllers       | `ros-noetic-ros-controllers`    |

To install a package from source clone (using git) or download the source files from where they are located (commonly hosted on GitHub) into the `src` folder of a ros catkin workspace and execute the [`catkin build`](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html) command. Also make sure to source the workspace after building new packages with `source devel/setup.bash`.

```console
cd /homw/fjp/git/diffbot/ros/  # Navigate to the workspace
catkin build              # Build all the packages in the workspace
ls build                  # Show the resulting build space
ls devel                  # Show the resulting devel space
```

Make sure to clone/download the source files suitable for the ROS distribtion you are using. If the sources are not available for the distribution you are working with, it is worth to try building anyway. Chances are that the package you want to use is suitable for multiple ROS distros. For example if a package states in its docs, that it is only available for [kinetic](http://wiki.ros.org/kinetic) it is possible that it will work with a ROS [noetic](http://wiki.ros.org/noetic) install.
{: .notice }

### Hardware Interface

See the [`include`](../diffbot_base/include) and [`src`](../diffbot_base/src) folders of this package for details on the hardware interface implementation.
More infos in this part of the documentation is coming soon (TODO). For now the section [ROS Integration: Control](https://fjp.at/projects/diffbot/ros-integration/#ros-control) gives more details and this [overview article about ROS Control](https://fjp.at/posts/ros/ros-control/).


### Launch File

To run a single controller_manager, the one from the `diffbot_base` package defined inside `difbot_base.cpp` use the 
launch file from [`diffbot_base/launch/diffbot.launch`](https://github.com/fjp/diffbot/blob/master/ros/src/diffbot_base/launch/diffbot.launch):

```xml
<!-- https://github.com/ros-controls/ros_controllers/tree/kinetic-devel/diff_drive_controller/test -->
<launch>
    <!-- Load DiffBot model -->
    <param name="robot_description"
	   command="$(find xacro)/xacro '$(find diffbot_description)/urdf/diffbot.xacro'"/>

    <node name="diffbot_base" pkg="diffbot_base" type="diffbot_base"/>

    <!-- Load controller config to the parameter server -->
    <rosparam command="load" 
              file="$(find diffbot_control)/config/diffbot_control.yaml"/>

    <!-- Load the controllers -->
    <node name="controller_spawner" pkg="controller_manager" type="spawner" respawn="false"
        output="screen" ns="diffbot" args="joint_state_controller
                                            mobile_base_controller"/>
</launch>
```

This will load the DiffBot robot description onto the parameter server which is required  for the hardware interface that gets created inside the next
node `diffbot_base`. It creates the hardware interface and instantiates a new controller manager in the `diffbot_base.cpp`.
Finally the `spawner` from the `controller_manager` package is used to initialize and start the controllers defined in the `diffbot_control/config/diffbot_control.yaml`. The last step in this launch file is required to get the controllers initialized and started.
Another way would be to use `controller_manager::ControllerManager::loadControllers()` inside the `diffbot_base.cpp`.


After launching this launch file on DiffBot (Raspberry Pi) with 

```console
roslaunch diffbot_base diffbot.launch
```

the following parameters are stored on the parameter server:

```console
$ rosparam list
/diffbot/hardware_interface/joints
/diffbot/joint_state_controller/extra_joints
/diffbot/joint_state_controller/publish_rate
/diffbot/joint_state_controller/type
/diffbot/mobile_base_controller/base_frame_id
/diffbot/mobile_base_controller/left_wheel
/diffbot/mobile_base_controller/pose_covariance_diagonal
/diffbot/mobile_base_controller/publish_rate
/diffbot/mobile_base_controller/right_wheel
/diffbot/mobile_base_controller/twist_covariance_diagonal
/diffbot/mobile_base_controller/type
/diffbot/mobile_base_controller/wheel_radius
/diffbot/mobile_base_controller/wheel_separation
/robot_description
/rosdistro
/roslaunch/uris/host_tensorbook__46157
/roslaunch/uris/host_ubuntu__33729
/rosversion
/run_id
```

To have a simulation showing DiffBot, the second step is to use the [`diffbot_gazebo/launch/diffbot_base.launch`](https://github.com/fjp/diffbot/blob/master/ros/src/diffbot_gazebo/launch/diffbot_base.launch) on the work pc:

```console
$ roslaunch diffbot_gazebo diffbot_base.launch
```

This will launch the gazebo simulation, which will can make use of the running controllers inside the controller manager too:

<figure>
    <a href="https://raw.githubusercontent.com/fjp/diffbot/master/docs/resources/ros_control_gazebo.png"><img src="https://raw.githubusercontent.com/fjp/diffbot/master/docs/resources/ros_control_gazebo.png"></a>
    <figcaption><a href="http://gazebosim.org/tutorials/?tut=ros_control" title="ROS Control with Gazebo">ROS Control with Gazebo</a> Overview.</figcaption>
</figure>


After launching the Gazebo simulation the controllers got uninitialized.
(It is assumed that the [`gazebo_ros_control`](https://github.com/ros-simulation/gazebo_ros_pkgs/tree/kinetic-devel/gazebo_ros_control) plugin that gets launched). Because of this the controllers have to be initialized and started again. For this the [`diffbot_base/launch/controllers.launch`](https://github.com/fjp/diffbot/blob/master/ros/src/diffbot_base/launch/controllers.launch) should be used.
This launch file is just loading and starting all controllers again. Note that using the `spawner` from the `controller_manager` package, like in the `diffbot_base/launch/diffbot.launch` results in an error. (TODO this needs some more testing).
