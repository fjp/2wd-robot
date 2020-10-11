## DiffBot Base Package

This package contains the so called hardware interface of DiffBot which represents the real hardware in software to work with 
[ROS Control](http://wiki.ros.org/ros_control). 

<figure>
    <a href="resources/ros_control_overview.png"><img src="resources/ros_control_overview.png"></a>
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
catkin create pkg diffbot_base --catkin-deps diff_drive_controller hardware_interface roscpp sensor_msgs diagnostic_updater                      
Creating package "diffbot_base" in "/home/fjp/git/diffbot/ros/src"...
Created file diffbot_base/package.xml
Created file diffbot_base/CMakeLists.txt
Created folder diffbot_base/include/diffbot_base
Created folder diffbot_base/src
Successfully created package files in /home/fjp/git/diffbot/ros/src/diffbot_base.
```

### Hardware Interface

See the `include` and `src` folders of this package for details on the hardware interface implementation.
More infos in the documentation is coming soon (TODO).


### Launch File


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

After launching this launch file with 

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

