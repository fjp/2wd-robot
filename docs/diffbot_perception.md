## Perception

To provide perception capabilities to your mobile robot you need a sensor like a RGB or RGB-D camera as hardware and software like 
[OpenCV](https://opencv.org/) or [PCL](https://pointclouds.org/).


When working with ROS we have to install required dependency using `ros-noetic-cv-bridge`. This will install additional dependencies like oopencv (`libopencv-dv`).
There is no need to install OpenCV from source or the Ubuntu binaries (deb package).
