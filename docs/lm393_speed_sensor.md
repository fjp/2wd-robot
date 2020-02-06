### LM393 Speed Sensor - Odometry

To measure how far the robot has driven, we use the [LM393 speed sensor](https://joy-it.net/en/products/SEN-Speed) from Joy-IT as [odometry](https://en.wikipedia.org/wiki/Odometry) sensor. 
First, we will create a ROS package with [`catkin create pkg PKG_NAME [--catkin-deps [DEP [DEP ...]]]`](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_create.html#catkin-create-pkg):

```bash
fjp@ubuntu:~/git/2wd-robot/ros/src$ catkin create pkg lm393_speed_sensor --catkin-deps rospy roscpp nav_msgs
Creating package "lm393_speed_sensor" in "/home/fjp/git/2wd-robot/ros/src"...
Created file lm393_speed_sensor/CMakeLists.txt
Created file lm393_speed_sensor/package.xml
Created folder lm393_speed_sensor/include/lm393_speed_sensor
Created folder lm393_speed_sensor/src
Successfully created package files in /home/fjp/git/2wd-robot/ros/src/lm393_speed_sensor.
```

The package depends on the two ROS [client libraries](http://wiki.ros.org/Client%20Libraries) [`rospy`](http://wiki.ros.org/rospy) and [`roscpp`](http://wiki.ros.org/roscpp). The current implementation uses python and the RPi.GPIO library for interrupts. To achieve more percise results, C++ should be used instead. 
To signalise the current pose of the robot in the odometry frame, the [`nav_msgs/Range`](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Range.html) message is used.

After connecting the signal pin of the sensor to [(physical) GPIO 15](https://pinout.xyz/pinout/pin15_gpio22#) of the Raspberry Pi 4 B and power it with 3.3V and ground, we can test its functionality with the following python script [`lm393_speed_sensor.py`]().


The following shows the truncated output of the `lm393_speed_sensor.py` script when the motors are spinning freely with full speed and using some force to slow the down. We see that the RPM values change. 

```bash
fjp@ubuntu:~/git/2wd-robot/ros/src/grove_ultrasonic_ranger/src$ sudo python ultrasonic.py
TODO
``` 
#### LM393 Speed Sensor Library

To use the LM393 speed sensor as a ROS node we wrap the sensor functionality in a class.
This provides an easy to extend interface for the speed sensor ([API](https://en.wikipedia.org/wiki/Application_programming_interface))
The code consists of a class LM393SpeedSensor which has two interrupt service routines (ISR) methods.
Using the RPi.GPIO interrupt capabilities, these ISR methods are used as callback functions when the sensor measures a falling
edge. This is the case when the rotary disk spins and the optocoupler measures a high to low signal due to the spinning disk. 

Executing the `lm393_speed_sensor.py` will result in the following output:

```bash
fjp@ubuntu:~/git/2wd-robot/ros/src/grove_ultrasonic_ranger/src$ sudo python lm393_speed_sensor.py 
TODO
```

#### ROS Node for LM393 Speed Sensor

ROS provides the [Odometry Message](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html) in the 
[nav_msgs header](https://wiki.ros.org/sensor_msgs). 
This message type can be used to write a wrapper that will act as a ROS node for the LM393 speed sensor.

To design this node we will send out measurements periodically over a topic of type `nav_msgs/Odometry`.
The code for this node is in [`speed_sensor.py`](https://github.com/fjp/2wd-robot/blob/master/ros/src/lm393_speed_sensor/src/speed_sensor.py).


After writing the node we need to build the packages in the workspace with [`catkin build`](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html).

```bash
fjp@ubuntu:~/git/2wd-robot/ros$ catkin build
TODO
```

As the final note of the build output suggests, we have to `source` the `setup.bash` files in the `devel` space.

```bash
fjp@ubuntu:~/git/2wd-robot/ros$ source devel/setup.bash
```

To make the `speed_sensor` node executable we have to modify the `speed_sensor.py` file:

```bash
fjp@ubuntu:~/git/2wd-robot/ros/src/grove_ultrasonic_ranger/src$ sudo chmod a+x speed_sensor.py
```

Then we can test the node using `rosrun`:

```bash
fjp@ubuntu:~/git/2wd-robot/ros$ sudo su
[sudo] password for fjp:
root@ubuntu:/home/fjp/git/2wd-robot/ros# source devel/setup.bash 
root@ubuntu:/home/fjp/git/2wd-robot/ros# rosrun lm393_speed_sensor speed_sensor.py 
```

This lets the node publish range messages which we can capture in another terminal window using `rostopic`.
First we use `rostopic list` to find the name of the topic we are interested in:

```bash
fjp@ubuntu:~/git/2wd-robot/ros$ rostopic list
TODO
```

We named our topic `/odom` which we can use with the `rostopic echo` command to see the published messages:

