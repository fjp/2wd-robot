## ROS Installation

The robot setup is supposed to run on Ubuntu 18.04 Bionic. [ROS Melodic]() is intended to run with this Ubuntu version.

From the [catkin tutorial](https://wiki.ros.org/catkin/Tutorials) here are the commands used to create the workspace:
Use [`catkin build`](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html) instead of [`catkin_make`](https://wiki.ros.org/catkin/commands/catkin_make).
[Here is why](https://robotics.stackexchange.com/questions/16604/ros-catkin-make-vs-catkin-build).

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
[build] No packages were found in the source space '/home/fjp/git/2wd-robot/ros/src'
[build] No packages to be built.
[build] Package table is up to date.
Starting  >>> catkin_tools_prebuild
Finished  <<< catkin_tools_prebuild                [ 10.0 seconds ]
[build] Summary: All 1 packages succeeded!
[build]   Ignored:   None.
[build]   Warnings:  None.
[build]   Abandoned: None.
[build]   Failed:    None.
[build] Runtime: 10.1 seconds total.
```
