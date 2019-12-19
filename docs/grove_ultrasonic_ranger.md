### Grove - Ultrasonic Ranger

To avoid obstacles the [Grove ultrasonic ranger](http://wiki.seeedstudio.com/Grove-Ultrasonic_Ranger/) from Seeed Studio is used. We will create a ROS package with [`catkin create pkg PKG_NAME [--catkin-deps [DEP [DEP ...]]]`](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_create.html#catkin-create-pkg):

```bash
fjp@ubuntu:~/git/2wd-robot/ros/src$ catkin create pkg grove_ultrasonic_ranger --catkin-deps rospy roscpp sensor_msgs
Creating package "grove_ultrasonic_ranger" in "/home/fjp/git/2wd-robot/ros/src"...
Created file grove_ultrasonic_ranger/CMakeLists.txt
Created file grove_ultrasonic_ranger/package.xml
Created folder grove_ultrasonic_ranger/include/grove_ultrasonic_ranger
Created folder grove_ultrasonic_ranger/src
```
The package depends on the two ROS [client libraries](http://wiki.ros.org/Client%20Libraries) [`rospy`](http://wiki.ros.org/rospy) and [`roscpp`](http://wiki.ros.org/roscpp). 
To signalise the current distance to obstacles the [`sensor_msgs/Range`](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Range.html) message is used.

After connecting the signal pin of the sensor to [(physical) GPIO 11](https://pinout.xyz/pinout/pin11_gpio17#) of the Raspberry Pi 4 B and power it with 5V and ground, we can test its functionality with the available 
python script [`ultrasonic.py`](https://github.com/Seeed-Studio/Grove-RaspberryPi/blob/master/Grove%20-%20Ultrasonic%20Ranger/ultrasonic.py) from Seed Studio.

Note that I renamed the script from `ultrasonic.py` to `seeedstudio_ultrasonic.py` because I use
a modified `ultrasonic.py` file as class to interface the ultrasonic ranger (Application Program Interface - API).
{: .notice-warning}

The following shows the truncated output of the `seeedstudio_ultrasonic.py` script when moving an obstacle in front of the sensor. We see that the distance value changes as expected. 

```bash
fjp@ubuntu:~/git/2wd-robot/ros/src/perception/src$ sudo python seeedstudio_ultrasonic.py 
SeeedStudio Grove Ultrasonic get data and print
Ultrasonic Measurement
Distance : 2.0 CM
Ultrasonic Measurement
Distance : 2.1 CM
Ultrasonic Measurement
Distance : 2.0 CM
Ultrasonic Measurement
Distance : 3.5 CM
Ultrasonic Measurement
Distance : 3.5 CM
Ultrasonic Measurement
Distance : 2.4 CM
Ultrasonic Measurement
Distance : 12.9 CM
Ultrasonic Measurement
Distance : 13.3 CM
Ultrasonic Measurement
Distance : 13.0 CM
Ultrasonic Measurement
Distance : 13.0 CM
Ultrasonic Measurement
Distance : 7.3 CM
Ultrasonic Measurement
Distance : 5.4 CM
Ultrasonic Measurement
Distance : 5.3 CM
Ultrasonic Measurement
Distance : 6.3 CM
Ultrasonic Measurement
Distance : 15.7 CM
Ultrasonic Measurement
Distance : 510.7 CM
Ultrasonic Measurement
Distance : 39.9 CM
Ultrasonic Measurement
Distance : 42.4 CM
Ultrasonic Measurement
Distance : 45.8 CM
Ultrasonic Measurement
Distance : 510.7 CM
Ultrasonic Measurement
Distance : 510.7 CM
Ultrasonic Measurement
Distance : 45.7 CM
Ultrasonic Measurement
Distance : 161.8 CM
Ultrasonic Measurement
Distance : 26.9 CM
Ultrasonic Measurement
Distance : 18.3 CM
Ultrasonic Measurement
Distance : 160.9 CM
Ultrasonic Measurement
Distance : 158.3 CM
Ultrasonic Measurement
Distance : 159.4 CM
...
``` 
#### Modified Ultrasonic Library

To use the ultrasonic ranger as a ROS node it is convenient to wrap the sensor functionality in a class.
This is why I wraped the core functionality of the [`ultrasonic.py`](https://github.com/Seeed-Studio/Grove-RaspberryPi/blob/master/Grove%20-%20Ultrasonic%20Ranger/ultrasonic.py) from Seeed Studio in a class `UltrasonicRanger`.


#### ROS Node for Ultrasonic Ranger

ROS provides the [Range Message](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Range.html) in the [sensor_msgs header](https://wiki.ros.org/sensor_msgs). This message type can be used to write a wrapper that will act as a ROS node for the Grove ultrasonic sensor.


### RPi Camera

