## DiffBot Bring Up Package

The bringup package is used to initialize the real hardware of DiffBot and to actually drive the robot around.
First the package is created using `catkin-tools`:

```console
fjp@diffbot:~/git/diffbot/ros/src$ catkin create pkg diffbot_bringup
Creating package "diffbot_bringup" in "/home/fjp/git/diffbot/ros/src"...
Created file diffbot_bringup/package.xml
Created file diffbot_bringup/CMakeLists.txt
Successfully created package files in /home/fjp/git/diffbot/ros/src/diffbot_bringup.
```


The package provides a `launch` folder which includes `minimal.launch` and `bringup.launch`.

The `minimal.launch` is used to load DiffBot's robot descripton and the controller configuration
onto the ROS parameter server using the launch file from the [`diffbot_base` package](https://fjp.at/projects/diffbot/ros-packages/base/). 
It will also setup the ROS [controller manager](http://wiki.ros.org/controller_manager) with 
[DiffBot's hardware interface](https://fjp.at/projects/diffbot/ros-packages/base/#hardware-interface).

For the motor driver the node `motor_driver.py` from the `grove_motor_driver` package is started.
And for the encoders rosserial communicates with the Teensy microcontroller to publish the encoder ticks. 

```xml
<launch>
    <!-- Including the following launch file from diffbot_base package will -->
    <!-- Load the robot description onto the parameter server -->
    <!-- Run the controller manager with DiffBot's hardware interface -->
    <!-- Load the controller config onto the parameter server -->
    <include file="$(find diffbot_base)/launch/diffbot.launch">
        <!-- arg name="model" value="$(arg model)" /-->
    </include>


    <!-- Motors -->
    <!-- -->
    <node name="motor_driver" pkg="grove_motor_driver" type="motor_driver.py" respawn="false"
        output="screen" ns="diffbot" />

    <!-- Encoders -->
    <!-- Run rosserial to connect with the Teensy 3.2 board connected to the motor encoders -->
    <node name="rosserial_teensy" pkg="rosserial_python" type="serial_node.py" respawn="false"
        output="screen" ns="diffbot" args="_port:=/dev/ttyACM0
                                            _baud:=115200"/>
</launch>
```

As mentioned, the ROS controller used for DiffBot is the [`diff_drive_controller`](http://wiki.ros.org/diff_drive_controller?distro=noetic). 
This controller publishes a transform message (see its [published topics](http://wiki.ros.org/diff_drive_controller?distro=noetic#Published_Topics)), 
via the `/tf` topic, between the `odom` frame and the frame configured in the [controller's configuration](http://wiki.ros.org/diff_drive_controller?distro=noetic#Complete_description) specified by the `base_frame_id`. 
In the case of DiffBot this is the `base_footprint`, a conventional link, defined in [REP-120](https://www.ros.org/reps/rep-0120.html#base-footprint), for mobile robots that specifies the robot's footprint.

Because this is the only transform published by `diff_drive_controller` another node is needed to publish rest of the link transformations.
It is the well known [`robot_state_publisher`](http://wiki.ros.org/robot_state_publisher), which uses the joint states published by the ROS controller [`joint_state_controller`](http://wiki.ros.org/joint_state_controller) (not to be confused with [`joint_state_publisher`](http://wiki.ros.org/joint_state_publisher) - it is not used here) to create the transforms
between the links.

To do this the `bringup.launch` includes the `minimal.launch` and then runs the `robot_state_publisher`:

```xml
<launch>
    <include file="$(find diffbot_bringup)/launch/minimal.launch">
        <!-- arg name="model" value="$(arg model)" /-->
    </include>

    <!-- Starting robot state publish which will publish tf -->
    <!-- This is needed to publish transforms between all links -->
    <!-- diff_drive_controller publishes only a single transfrom between odom and base_footprint -->
    <!-- The robot_state_publisher reads the joint states published by ros control's joint_state_controller -->
    <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"
        output="screen" ns="diffbot" />
</launch>
```
