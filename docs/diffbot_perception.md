## Perception

To provide perception capabilities to your mobile robot you need a sensor like a RGB or RGB-D camera as hardware and software like 
[OpenCV](https://opencv.org/) or [PCL](https://pointclouds.org/).


When working with ROS we have to install required dependency using `ros-noetic-vision-opencv`. This will install additional dependencies like OpenCV (`libopencv-dv`) and `ros-noetic-cv-bridge`. There is no need to install OpenCV from source or the Ubuntu binaries (deb package).

If you want to use OpenCV without ROS you should consider installing it from [source using CMake](https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html). 
This will allow you to configure what features should be installed, e.g., example code.
{: .notice }
