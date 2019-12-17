## Perception - Catkin package

The perception ROS package is created with `catkin create pkg PKG_NAME`:

```bash
fjp@ubuntu:~/git/2wd-robot/ros/src$ catkin create pkg perception
Creating package "perception" in "/home/fjp/git/2wd-robot/ros/src"...
Created file perception/CMakeLists.txt
Created file perception/package.xml
Successfully created package files in /home/fjp/git/2wd-robot/ros/src/perception.
```

### Grove - Ultrasonic Ranger

To avoid obstacles the [Grove ultrasonic ranger](http://wiki.seeedstudio.com/Grove-Ultrasonic_Ranger/) from Seeed Studio is used. After connecting the signal pin of the sensor to [(physical) GPIO 11](https://pinout.xyz/pinout/pin11_gpio17#) of the Raspberry Pi 4 B and power it with 5V and ground, we can test its functionality with the available 
[python script](https://github.com/Seeed-Studio/Grove-RaspberryPi/blob/master/Grove%20-%20Ultrasonic%20Ranger/ultrasonic.py) from Seed Studio.

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
This is why I wraped the core functionality of the `ultrasonic.py` from Seeed Studio in a class `UltrasonicRanger`.


#### ROS Node for Ultrasonic Ranger

ROS provides the [Ranger Message](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Range.html) in the [sensor_msgs header](https://wiki.ros.org/sensor_msgs). This message type can be used to write a wrapper that will act as a ROS node for the Grove ultrasonic sensor.


### RPi Camera

