## DiffBot Messages Package


As mentioned before, the nodes in ROS communicate with each other by publishing [messages](http://wiki.ros.org/Messages) to [topics](http://wiki.ros.org/Topics). 
ROS provides the [`std_msgs`](http://wiki.ros.org/std_msgs) package that includes ROS' common message types to represent primitive data types (see the ROS [msg specification](http://wiki.ros.org/msg) for primitive types) and other basic message constructs, such as multiarrays. 
Note howerver, the following from the [`std_msgs` documentation](http://wiki.ros.org/std_msgs):

!!! quote
    The types in `std_msgs` do not convey semantic meaning about their contents: every message simply has a field called "data". 
    Therefore, while the messages in this package can be useful for quick prototyping, they are NOT intended for "long-term" usage. 
    For ease of documentation and collaboration, we recommend that existing messages be used, or new messages created, that provide meaningful field name(s).


Therefore, we create a package that contains message definitions specific to DiffBot. 
The following command uses `catkin-tools` to create the `diffbot_msgs` package:

```console
ros_ws/src$ catkin create pkg diffbot_msgs --catkin-deps message_generation std_msgs                                                           
Creating package "diffbot_msgs" in "/home/fjp/git/ros_ws/src"...
WARNING: Packages with messages or services should depend on both message_generation and message_runtime
Created file diffbot_msgs/package.xml
Created file diffbot_msgs/CMakeLists.txt
Successfully created package files in /home/fjp/git/ros_ws/src/diffbot_msgs.
```

!!! note
    The following is based on [ROS Tutorials Creating Msg And Srv](https://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv#Creating_a_msg).
    In this tutorial you can find the required configurations for the `package.xml` and `CMakeLists.txt`.

Currently there is no encoder message definition in ROS (see the [`sensor_msgs`](https://wiki.ros.org/sensor_msgs) package) 
which is why a dedicated message is created for the encoders. For this, a simple [msg](http://wiki.ros.org/msg) description file,
named `Encoder.msg` is created in the `msg/` subdirectory of this `diffbot_msgs` package:


```
# This is a message to hold number of ticks from Encoders
Header header

# Use an array of size two of type int32 for the two encoders.
# int32 is used instead of int64 because it is not supporte by Arduino/Teensy.
# An overflow is also unlikely with the encoders of the DG01D-E 
# motor with encoder because of its low encoder resolution
int32[2] encoders
```

The message includes the message type [`Header`](http://docs.ros.org/en/api/std_msgs/html/msg/Header.html) 
(see also [Header msg](http://wiki.ros.org/msg#headerSect)) which includes common metadata fileds such as timestamp that is automatically 
set by [ROS client libraries](http://wiki.ros.org/Client%20Libraries).

Having this encoder message description gives semantic meaning to the encoder messages 
and for example avoids having two separate int32 publishers for each encoder.
Combining the encoder message into a single one alleviates additional timing problems.


There exists also the [`common_msgs`](https://wiki.ros.org/common_msgs) meta package for common, generic robot-specific message types.
From the `common_msgs` DiffBot uses for example the [`nav_msgs`](http://wiki.ros.org/nav_msgs) for navigation with the [navigation stack](http://wiki.ros.org/navigation). Other relevant message definitions are the [`sensor_msgs/Imu`](http://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html) 
and `sensor_msgs/LaserScan`](http://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html), 
where both are definitions from the [`sensor_msgs`](https://wiki.ros.org/sensor_msgs) package.


### Using rosmsg

After building the package and its messages using `catkin build` let's make sure that ROS can see it using the rosmsg show command.

```console
$ rosmsg show diffbot_msgs/Encoder
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
int32[2] encoders

```


### ROSSerial

The generated messages in this packages are used on the Teensy microcontroller, which is using [`rosserial`](http://wiki.ros.org/rosserial).
Integrating these messages requires the following steps.

- Generate rosserial libraries in a temporary folder using the [`make_libraries`](http://wiki.ros.org/rosserial/Tutorials/Adding%20Other%20Messages) script:

```console
rosrun rosserial_client make_libraries ~/Arduino/tmp/
``` 

- Copy the generated `~/Arduino/tmp/diffbot_msgs` message folder to the `src` folder of the `rosserial` Arduino library.
  When `rosserial` was installed with the Arduino Library Manager, the location is `~/Arduino/libraries/Rosserial_Arduino_Library/`.


### Usage

The new messages, specific to DiffBot, can be used by including the generated header, for example `#include <diffbot_msgs/Encoder.h>`.



### References

- [Tutorials Arduino IDE Setup](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup)