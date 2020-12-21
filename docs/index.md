# Welcome to DiffBot Documentation

This project guides you on how to build an autonomous two wheel differential drive robot. [![image](https://img.shields.io/github/stars/fjp/diffbot?style=social)](https://github.com/fjp/diffbot)
The robot is equipped with a [Raspberry Pi 4 B](https://de.aliexpress.com/item/32858825148.html?spm=a2g0o.productlist.0.0.5d232e8bvlKM7l&algo_pvid=2c45d347-5783-49a6-a0a8-f104d0b78232&algo_expid=2c45d347-5783-49a6-a0a8-f104d0b78232-0&btsid=0100feb4-37d7-453a-8ff8-47a0e2fbdef7&ws_ab_test=searchweb0_0,searchweb201602_9,searchweb201603_52) running [ROS Noetic](http://wiki.ros.org/noetic) middleware on Ubuntu Mate 20.04.
With a motor driver and two actuators it can drive autonomously to a desired location while sensing its environment using sensors, 
such as a camera and an ultrasonic ranger to avoid obstacles. Speed sensors combined with an inertial measurement unit (IMU) are used for localization.
The project is split into multiple parts, to adress the following main aspects of the robot.

- [Part list](/projects/diffbot/components/) and the theory behind the parts.
- [Assembly](/projects/diffbot/assembly/) of the robot platform and the components.
- Raspberry Pi 4 B setup using ROS Noetic, which will be the brain of the robot.
- [Modeling the Robot](/projects/diffbot/URDF) in Blender and URDF to simulate it in Gazebo.
- ROS packages and nodes: 
  - Hardware drivers to interact with the hardware components
  - High level nodes for perception, navigation, localization and control.

Use the menu on the left to learn more about the ROS packages and other components of the robot.

!!! note
    Using a [Jetson Nano](https://developer.nvidia.com/embedded/jetson-nano-developer-kit) instead of a Raspberry Pi is also possible.


## Source Code

The source code for this project can be found in [this GitHub repository](https://github.com/fjp/diffbot).

## References

Helpful resources to bring your own robots into ROS are:

- Understand [ROS Concepts](https://wiki.ros.org/ROS/Concepts)
- Follow [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials) such as [Using ROS on your custom Robot](http://wiki.ros.org/ROS/Tutorials#Using_ROS_on_your_custom_Robot)
- Books:
  - [**Robot Operating System (ROS) for Absolute Beginners**](https://link.springer.com/book/10.1007/978-1-4842-3405-1) from Apress by [Lentin Joseph](https://lentinjoseph.com/)
  - [**Programming Robots with ROS** A Practical Introduction to the Robot Operating System](http://shop.oreilly.com/product/0636920024736.do) from O'Reilly Media
  - [**Mastering ROS for Robotics Programming** Second Edition](https://www.packtpub.com/eu/hardware-and-creative/mastering-ros-robotics-programming-second-edition) from Packt
  - [**Elements of Robotics** Robots and Their Applications](https://www.springer.com/de/book/9783319625324) from Springer
