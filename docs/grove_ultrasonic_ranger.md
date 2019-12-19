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


The following shows the truncated output of the `ultrasonic.py` script when moving an obstacle in front of the sensor. We see that the distance value changes as expected. 

```bash
fjp@ubuntu:~/git/2wd-robot/ros/src/grove_ultrasonic_ranger/src$ sudo python ultrasonic.py
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
#### Modified GroveUltrasonicRanger Library

To use the ultrasonic ranger as a ROS node it is convenient to wrap the sensor functionality in a class.
This provides an easy to extend interface for the ultrasonic ranger ([API](https://en.wikipedia.org/wiki/Application_programming_interface))
Therefore I copied the core functionality of the [`ultrasonic.py`](https://github.com/Seeed-Studio/Grove-RaspberryPi/blob/master/Grove%20-%20Ultrasonic%20Ranger/ultrasonic.py) script from Seeed Studio in a class named [`GroveUltrasonicRanger`](https://github.com/fjp/2wd-robot/blob/master/ros/src/grove_ultrasonic_ranger/src/grove_ultrasonic_ranger.py).

Executing the `grove_ultrasonic_ranger.py` will result in the following output:

```bash
fjp@ubuntu:~/git/2wd-robot/ros/src/grove_ultrasonic_ranger/src$ sudo python grove_ultrasonic_ranger.py 
SeeedStudio Grove Ultrasonic get data and print
Distance : 0.051 m
Distance : 0.069 m
Distance : 0.098 m
Distance : 0.131 m
Distance : 0.153 m
Distance : 0.172 m
Distance : 0.207 m
Distance : 0.210 m
Distance : 0.234 m
Distance : 0.256 m
GPIO.cleanup()
GPIO.cleanup() done
```

#### ROS Node for Ultrasonic Ranger

ROS provides the [Range Message](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Range.html) in the [sensor_msgs header](https://wiki.ros.org/sensor_msgs). This message type can be used to write a wrapper that will act as a ROS node for the Grove ultrasonic sensor.

To design this node we will send out measurements periodically over a topic of type `sensor_msgs/Range`.
The code for this node is in [`ranger.py`](https://github.com/fjp/2wd-robot/blob/master/ros/src/grove_ultrasonic_ranger/src/ranger.py).


After writing the node we need to build the packages in the workspace with `catkin build`.

```bash
fjp@ubuntu:~/git/2wd-robot/ros$ catkin build
----------------------------------------------------------------
Profile:                     default
Extending:             [env] /opt/ros/melodic
Workspace:                   /home/fjp/git/2wd-robot/ros
----------------------------------------------------------------
Build Space:        [exists] /home/fjp/git/2wd-robot/ros/build
Devel Space:        [exists] /home/fjp/git/2wd-robot/ros/devel
Install Space:      [unused] /home/fjp/git/2wd-robot/ros/install
Log Space:         [missing] /home/fjp/git/2wd-robot/ros/logs
Source Space:       [exists] /home/fjp/git/2wd-robot/ros/src
DESTDIR:            [unused] None
----------------------------------------------------------------
Devel Space Layout:          linked
Install Space Layout:        None
----------------------------------------------------------------
Additional CMake Args:       None
Additional Make Args:        None
Additional catkin Make Args: None
Internal Make Job Server:    True
Cache Job Environments:      False
----------------------------------------------------------------
Whitelisted Packages:        None
Blacklisted Packages:        None
----------------------------------------------------------------
Workspace configuration appears valid.

NOTE: Forcing CMake to run for each package.
----------------------------------------------------------------
[build] Found '1' package in 0.0 seconds.                                                                                                
[build] Updating package table.                                                                                                           
Starting  >>> catkin_tools_prebuild                                                                                                       
Finished  <<< catkin_tools_prebuild                  [ 9.9 seconds ]                                                                                                                                                                                         
Starting  >>> grove_ultrasonic_ranger                                                                                                                                                                                                                                                                                                                                                         
Finished  <<< grove_ultrasonic_ranger                [ 12.0 seconds ]                                                                     
[build] Summary: All 2 packages succeeded!                                                                                                
[build]   Ignored:   None.                                                                                                                
[build]   Warnings:  None.                                                                                                                
[build]   Abandoned: None.                                                                                                                
[build]   Failed:    None.                                                                                                                
[build] Runtime: 21.9 seconds total.                                                                                                      
[build] Note: Workspace packages have changed, please re-source setup files to use them.
```

As the final note of the build output suggests, we have to `source` the `setup.bash` files in the `devel` space.

```bash
fjp@ubuntu:~/git/2wd-robot/ros$ source devel/setup.bash
```

To make the `ranger` node executable we have to modify the `ranger.py` file:

```bash
fjp@ubuntu:~/git/2wd-robot/ros/src/grove_ultrasonic_ranger/src$ sudo chmod a+x ranger.py
```

Then we can test the node using `rosrun`:

```bash
fjp@ubuntu:~/git/2wd-robot/ros$ sudo su
[sudo] password for fjp:
root@ubuntu:/home/fjp/git/2wd-robot/ros# source devel/setup.bash 
root@ubuntu:/home/fjp/git/2wd-robot/ros# rosrun grove_ultrasonic_ranger ranger.py 
Distance : 1.617 m
Distance : 1.617 m
Distance : 1.617 m
Distance : 0.108 m
Distance : 0.092 m
Distance : 0.099 m
```

This lets the node publishe range messages which we can capture in another terminal window using `rostopic`.
First we use `rostopic list` to find the name of the topic we are interested in:

```bash
fjp@ubuntu:~/git/2wd-robot/ros$ rostopic list
/distance
/rosout
/rosout_agg
```

We named our topic `/distance` which we can use with the `rostopic echo` command to see the published messages:

```bash
fjp@ubuntu:~/git/2wd-robot/ros$ rostopic echo /distance
header: 
  seq: 1
  stamp: 
    secs: 1576778377
    nsecs: 746809959
  frame_id: "ranger_distance"
radiation_type: 0
field_of_view: 0.261799007654
min_range: 0.019999999553
max_range: 3.5
range: 1.61674261093
---
header: 
  seq: 2
  stamp: 
    secs: 1576778378
    nsecs: 459048986
  frame_id: "ranger_distance"
radiation_type: 0
field_of_view: 0.261799007654
min_range: 0.019999999553
max_range: 3.5
range: 1.61261284351
---
header: 
  seq: 3
  stamp: 
    secs: 1576778379
    nsecs: 172172069
  frame_id: "ranger_distance"
radiation_type: 0
field_of_view: 0.261799007654
min_range: 0.019999999553
max_range: 3.5
range: 1.61657905579
---
header: 
  seq: 4
  stamp: 
    secs: 1576778379
    nsecs: 884002923
  frame_id: "ranger_distance"
radiation_type: 0
field_of_view: 0.261799007654
min_range: 0.019999999553
max_range: 3.5
range: 1.61277639866
---
header: 
  seq: 5
  stamp: 
    secs: 1576778380
    nsecs: 596549034
  frame_id: "ranger_distance"
radiation_type: 0
field_of_view: 0.261799007654
min_range: 0.019999999553
max_range: 3.5
range: 1.67693090439
---
```


#### Informational Distance Measurements

To provide additional information when an obstacle is too close or the robot has no obstacle in front of it we use
[REP-117](https://www.ros.org/reps/rep-0117.html) as guideline.
