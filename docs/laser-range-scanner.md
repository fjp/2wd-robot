## Laser Range Scanner

For SLAM (simultaneous localization and mapping) and navigation a laser range scanner is a cruical 
requirement to use the ROS navigation stack (see also `diffbot_navigation`). 
DiffBot uses the 360 field of view [RPLidar A2 M8](https://www.slamtec.com/en/Lidar/A2) with 12 meter range.


### Mounting

When installing the RPLidar on DiffBot make sure it is located high enough to avoid occluding the laser.
It is also important to mount the laser in the correct location and that its direction and orientation matches the `origin` alignment in the robot description.
In the figure below we can see, that the side with the communication cable is pointing in the positive x-direction.
In case the cable should be mounted in the opposite direction, it is necessary to adapt the origin of the laser link in the robot description.

<figure>
    <a href="https://raw.githubusercontent.com/robopeak/rplidar_ros/master/rplidar_A2.png"><img src="https://raw.githubusercontent.com/robopeak/rplidar_ros/master/rplidar_A2.png"></a>
    <figcaption>Installation manual for RPLidar A2 (source: [robopeak/rplidar_ros/](https://github.com/robopeak/rplidar_ros/wiki/How-to-use-rplidar).</figcaption>
</figure>
