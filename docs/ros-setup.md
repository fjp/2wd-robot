## ROS Installation

The robot setup is supposed to run on Ubuntu Mate 20.04 Focal Fossa. [ROS Noetic](http://wiki.ros.org/melodic) is intended to run with this Ubuntu version.

Another program that is required to run ROS nodes written with the `rospy` client library is `python-is-python3`. Install it with:

```
sudo apt install python-is-python3
```

## Build Tool: `catkin_tools`

To work with ROS we will use [`catkin_tools`](https://catkin-tools.readthedocs.io/en/latest/index.html) 
instead of [`catkin_make`](http://wiki.ros.org/catkin/commands/catkin_make). `catkin_tools` provide commands such as [`catkin build`](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html) which we will use instead of [`catkin_make`](https://wiki.ros.org/catkin/commands/catkin_make) because the `catkin_tools` are more actively developed than `catkin_make`
[ref](https://robotics.stackexchange.com/questions/16604/ros-catkin-make-vs-catkin-build). 

Follow the [instructions to insall](https://catkin-tools.readthedocs.io/en/latest/installing.html#installing-on-ubuntu-with-apt-get) `catkin_tools` from a Ubuntu package repository. After sucessfully installing `catkin_tools` we can create and initialize a workspace (called `ros` for this project) with the [commands listed in the build_tools documentation](https://catkin-tools.readthedocs.io/en/latest/quick_start.html):

Note that we already `source`d the `setup.bash` while following the [ROS installation instructions](http://wiki.ros.org/melodic/Installation/Ubuntu).
{: .notice }


```bash
fjp@ubuntu:~/git/2wd-robot/ros$ mkdir -p ~/git/2wd-robot/ros/src    # Make a new workspace and source space
fjp@ubuntu:~/git/2wd-robot/ros$ cd ~/git/2wd-robot/ros              # Navigate to the workspace root
fjp@ubuntu:~/git/2wd-robot/ros$ catkin init                         # Initialize it with a hidden marker file
Initializing catkin workspace in `/home/fjp/git/2wd-robot/ros`.
----------------------------------------------------------------
Profile:                     default
Extending:             [env] /opt/ros/melodic
Workspace:                   /home/fjp/git/2wd-robot/ros
----------------------------------------------------------------
Build Space:       [missing] /home/fjp/git/2wd-robot/ros/build
Devel Space:       [missing] /home/fjp/git/2wd-robot/ros/devel
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
----------------------------------------------------------------
```

## Command Overview of `catkin_tools`

To [create packages](https://catkin-tools.readthedocs.io/en/latest/quick_start.html#adding-packages-to-the-workspace), which will be covered in the next posts in more depth, we will use [`catkin create pkg PKG_NAME`](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_create.html#catkin-create-pkg).

Building the workspace is done with [`catkin build`](https://catkin-tools.readthedocs.io/en/latest/quick_start.html#building-the-workspace).

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

Finally the newly built packages have to be [loaded in the environment](https://catkin-tools.readthedocs.io/en/latest/quick_start.html#loading-the-workspace-environment) using `source`.

```bash
fjp@ubuntu:~/git/2wd-robot/ros$ source ~/git/2wd-robot/ros/devel/setup.bash # Load the workspace's environment
```

## Resources

Although the [catkin tutorial](https://wiki.ros.org/catkin/Tutorials) uses `catkin_make` it provides a helpful guide to create a workspace
