## DiffBot Base Package

This package contains the so called hardware interface of DiffBot which represents the real hardware in software to work with 
[ROS Control](http://wiki.ros.org/ros_control). 

<figure>
    <a href="https://raw.githubusercontent.com/fjp/diffbot/master/docs/resources/ros_control_overview.png"><img src="https://raw.githubusercontent.com/fjp/diffbot/master/docs/resources/ros_control_overview.png"></a>
    <figcaption><a href="http://wiki.ros.org/ros_control#Overview" title="ROS Control">ROS Control</a> Overview.</figcaption>
</figure>

In the simpleste case all that is needed in this package is to write a class that inherits from `hardware_interface::RobotHW` and provide a launch
file. The launch file will 

- Load the robot description from `diffbot_description` to the paramter server
- Run the hardware interface of this package `diffbot_base`
- Load the controller configuration yaml from the `diffbot_control` package to the parameter server
- Load the controllers with the [controller manager](http://wiki.ros.org/controller_manager?distro=noetic)

### diffbot_base Package

The `diffbot_base` package is created with `catkin-tools`:

```console
fjp@diffbot:/home/fjp/catkin_ws/src$ catkin create pkg diffbot_base --catkin-deps diff_drive_controller hardware_interface roscpp sensor_msgs rosparam_shortcuts                 
Creating package "diffbot_base" in "/home/fjp/catkin_ws/src"...
Created file diffbot_base/package.xml
Created file diffbot_base/CMakeLists.txt
Created folder diffbot_base/include/diffbot_base
Created folder diffbot_base/src
Successfully created package files in /home/fjp/catkin_ws/src/diffbot_base.
```

To work with this package the specified dependencies must be installed either using the available Ubuntu/Debian packages for ROS Noetic or they have to be built from source first. The following table lists the dependencies that we have to install because they are not already part of the ROS Noetic desktop full installation. Refer to the section [ROS Noetic Setup](https://fjp.at/projects/diffbot/ros-noetic/) for how this was done. 

| Dependency                    | Source                                                | Ubuntu/Debian Package            |
|:-----------------------------:|:-----------------------------------------------------:|:--------------------------------:|
| `rosparam_shortcuts`          | https://github.com/PickNikRobotics/rosparam_shortcuts | `ros-noetic-rosparam-shortcuts` |
| `hardware_interface`          | https://github.com/ros-controls/ros_control           | `ros-noetic-ros-control`        |
| `diff_drive_controller`       | https://github.com/ros-controls/ros_controllers       | `ros-noetic-ros-controllers`    |

To install a package from source clone (using git) or download the source files from where they are located (commonly hosted on GitHub) into the `src` folder of a ros catkin workspace and execute the [`catkin build`](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html) command. Also make sure to source the workspace after building new packages with `source devel/setup.bash`.

```console
cd /homw/fjp/git/diffbot/ros/  # Navigate to the workspace
catkin build              # Build all the packages in the workspace
ls build                  # Show the resulting build space
ls devel                  # Show the resulting devel space
```

Make sure to clone/download the source files suitable for the ROS distribtion you are using. If the sources are not available for the distribution you are working with, it is worth to try building anyway. Chances are that the package you want to use is suitable for multiple ROS distros. For example if a package states in its docs, that it is only available for [kinetic](http://wiki.ros.org/kinetic) it is possible that it will work with a ROS [noetic](http://wiki.ros.org/noetic) install.
{: .notice }

### Hardware Interface

See the [`include`](../diffbot_base/include) and [`src`](../diffbot_base/src) folders of this package and the details on the hardware interface implementation.
For more details on the hardware interface also refer to the section [ROS Integration: Control](https://fjp.at/projects/diffbot/ros-integration/#ros-control), it gives more details and also this [overview article about ROS Control](https://fjp.at/posts/ros/ros-control/).

The hardware interface provides an interface between the real robot hardware and the controllers provided by ROS Control (or even custom controllers).
DiffBot works with the `diff_drive_controller` that is configured in the `diffbot_control` package, which is also relevant for the simulation in Gazebo.
Remember that the simulation uses the `gazebo_ros_control` package to communicate with the `diff_drive_controller`. For the real robot hardware,
ROS Control uses an instance of type [`hardware_interface::RobotHW`](http://docs.ros.org/en/noetic/api/hardware_interface/html/c++/) that is passed to the `controller_manager` to handle the resources, meaning that the actuated robot joints are not in use by multiple controllers that might be loaded.


The skeleton of DiffBot's hardware interface looks like following, where the constructor is used to read loaded configuration values from 
the robot's description from the ROS parameter server:

```cpp
namespace diffbot_base
{
    DiffBotHWInterface::DiffBotHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
        : name_("hardware_interface")
        , nh_(nh)
    { 
        // Initialization of the robot's resources (joints, sensors, actuators) and
        // interfaces can be done here or inside init().
        // E.g. parse the URDF for joint names & interfaces, then initialize them
        // Check if the URDF model needs to be loaded
        if (urdf_model == NULL)
            loadURDF(nh, "robot_description");
        else
            urdf_model_ = urdf_model;

        // Load rosparams
        ros::NodeHandle rpnh(nh_, name_);
        std::size_t error = 0;
        // Code API of rosparam_shortcuts:
        // http://docs.ros.org/en/noetic/api/rosparam_shortcuts/html/namespacerosparam__shortcuts.html#aa6536fe0130903960b1de4872df68d5d
        error += !rosparam_shortcuts::get(name_, rpnh, "joints", joint_names_);
        error += !rosparam_shortcuts::get(name_, nh_, "mobile_base_controller/wheel_radius", wheel_radius_);
        error += !rosparam_shortcuts::get(name_, nh_, "mobile_base_controller/linear/x/max_velocity", max_velocity_);
        rosparam_shortcuts::shutdownIfError(name_, error);

        wheel_diameter_ = 2.0 * wheel_radius_;
        //max_velocity_ = 0.2; // m/s
        // ros_control RobotHW needs velocity in rad/s but in the config its given in m/s
        max_velocity_ = linearToAngular(max_velocity_);

        // Setup publisher for the motor driver 
        pub_left_motor_value_ = nh_.advertise<std_msgs::Int32>("motor_left", 1);
        pub_right_motor_value_ = nh_.advertise<std_msgs::Int32>("motor_right", 1);

        // Setup subscriber for the wheel encoders
        sub_left_encoder_ticks_ = nh_.subscribe("ticks_left", 1, &DiffBotHWInterface::leftEncoderTicksCallback, this);
        sub_right_encoder_ticks_ = nh_.subscribe("ticks_right", 1, &DiffBotHWInterface::rightEncoderTicksCallback, this);

        // Initialize the hardware interface
        init(nh_, nh_);
    }

 
    bool DiffBotHWInterface::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh)
    {
        ROS_INFO("Initializing DiffBot Hardware Interface ...");
        num_joints_ = joint_names_.size();
        ROS_INFO("Number of joints: %d", (int)num_joints_);
        std::array<std::string, NUM_JOINTS> motor_names = {"left_motor", "right_motor"};
        for (unsigned int i = 0; i < num_joints_; i++)
        {
            // Create a JointStateHandle for each joint and register them with the 
            // JointStateInterface.
            hardware_interface::JointStateHandle joint_state_handle(joint_names_[i],
                                                                    &joint_positions_[i], 
                                                                    &joint_velocities_[i],
                                                                    &joint_efforts_[i]);
            joint_state_interface_.registerHandle(joint_state_handle);

            // Create a JointHandle (read and write) for each controllable joint
            // using the read-only joint handles within the JointStateInterface and 
            // register them with the JointVelocityInterface.
            hardware_interface::JointHandle joint_handle(joint_state_handle, &joint_velocity_commands_[i]);
            velocity_joint_interface_.registerHandle(joint_handle);

            // Initialize joint states with zero values
            joint_positions_[i] = 0.0;
            joint_velocities_[i] = 0.0;
            joint_efforts_[i] = 0.0; // unused with diff_drive_controller

            joint_velocity_commands_[i] = 0.0;

            // Initialize the pid controllers for the motors using the robot namespace
            std::string pid_namespace = "pid/" + motor_names[i];
            ROS_INFO_STREAM("pid namespace: " << pid_namespace);
            ros::NodeHandle nh(root_nh, pid_namespace);
            // TODO implement builder pattern to initialize values otherwise it is hard to see which parameter is what.
            pids_[i].init(nh, 0.0, 10.0, 1.0, 1.0, 0.0, 0.0, false, -max_velocity_, max_velocity_);
            pids_[i].setOutputLimits(-max_velocity_, max_velocity_);
        }

        // Register the JointStateInterface containing the read only joints
        // with this robot's hardware_interface::RobotHW.
        registerInterface(&joint_state_interface_);

        // Register the JointVelocityInterface containing the read/write joints
        // with this robot's hardware_interface::RobotHW.
        registerInterface(&velocity_joint_interface_);

        ROS_INFO("... Done Initializing DiffBot Hardware Interface");
        return true;
    }

    // The read method is part of the control loop cycle (read, update, write) and is used to 
    // populate the robot state from the robot's hardware resources (joints, sensors, actuators). 
    // This method should be called before controller_manager::ControllerManager::update() and write.
    void DiffBotHWInterface::read(const ros::Time& time, const ros::Duration& period)
    {
        ros::Duration elapsed_time = period;

        // Read from robot hw (motor encoders)
        // Fill joint_state_* members with read values
        double wheel_angles[2];
        double wheel_angle_deltas[2];
        for (std::size_t i = 0; i < num_joints_; ++i)
        {
            wheel_angles[i] = ticksToAngle(encoder_ticks_[i]);
            //double wheel_angle_normalized = normalizeAngle(wheel_angle);
            wheel_angle_deltas[i] = wheel_angles[i] - joint_positions_[i];
            
            joint_positions_[i] += wheel_angle_deltas[i];
            joint_velocities_[i] = wheel_angle_deltas[i] / period.toSec();
            joint_efforts_[i] = 0.0; // unused with diff_drive_controller
        }
    }

    // The write method is part of the control loop cycle (read, update, write) and is used to 
    // send out commands to the robot's hardware resources (joints, actuators). 
    // This method should be called after read and controller_manager::ControllerManager::update.
    void DiffBotHWInterface::write(const ros::Time& time, const ros::Duration& period)
    {
        ros::Duration elapsed_time = period;
        // Write to robot hw
        // joint velocity commands from ros_control's RobotHW are in rad/s
        // Convert the velocity command to a percentage value for the motor
        // This maps the velocity to a percentage value which is used to apply
        // a percentage of the highest possible battery voltage to each motor.
        std_msgs::Int32 left_motor;
        std_msgs::Int32 right_motor;

        double output_left = pids_[0](joint_velocities_[0], joint_velocity_commands_[0], period);
        double output_right = pids_[1](joint_velocities_[1], joint_velocity_commands_[1], period);

        left_motor.data = output_left / max_velocity_ * 100.0;
        right_motor.data = output_right / max_velocity_ * 100.0;
	
	// Publish the PID computed motor commands to the left and right motors
	pub_left_motor_value_.publish(left_motor);
        pub_right_motor_value_.publish(right_motor);
    }

    // Process updates from encoders using a subscriber
    void DiffBotHWInterface::leftEncoderTicksCallback(const std_msgs::Int32::ConstPtr& msg)
    {
        encoder_ticks_[0] = msg->data;
        ROS_DEBUG_STREAM_THROTTLE(1, "Left encoder ticks: " << msg->data);
    }

    void DiffBotHWInterface::rightEncoderTicksCallback(const std_msgs::Int32::ConstPtr& msg)
    {
        encoder_ticks_[1] = msg->data;
        ROS_DEBUG_STREAM_THROTTLE(1, "Right encoder ticks: " << msg->data);
    }

    double DiffBotHWInterface::ticksToAngle(const int &ticks) const
    {
        // Convert number of encoder ticks to angle in radians
        double angle = (double)ticks * (2.0*M_PI / 542.0);
        ROS_DEBUG_STREAM_THROTTLE(1, ticks << " ticks correspond to an angle of " << angle);
	return angle;
    }

};
```

The functions above are designed to give the controller manager (and the controllers inside the controller manager) access to the joint state of custom robot, 
and to command it. When the controller manager runs, the controllers will read from the pos, vel and eff variables of the custom robot hardware interface, and the controller will write the desired command into the cmd variable. It's mandatory to make sure the pos, vel and eff variables always have the latest joint state available, and to make sure that whatever is written into the cmd variable gets executed by the robot. This can be done by implementing hardware_interface::RobotHW::read() and a hardware_interface::RobotHW::write() methods.



The main node that will be executed uses the `controller_manager` to operate the so called control loop. In the case of  DiffBot a simple example looks like 
the following, refer to the [`diffbot_base.cpp`](https://github.com/fjp/diffbot/blob/master/diffbot_base/src/diffbot_base.cpp) for the complete implementation:

```cpp
#include <ros/ros.h>
#include <diffbot_base/diffbot_hw_interface.h>
#include <controller_manager/controller_manager.h>
 
int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "diffbot_hw_interface");
    ros::NodeHandle nh;
    
    // Create an instance of your robot so that this instance knows about all 
    // the resources that are available.
    diffbot_base::DiffBotHWInterface diffBot(nh);
 
    // Create an instance of the controller manager and pass it the robot, 
    // so that it can handle its resources.
    controller_manager::ControllerManager cm(&diffBot);
    
    // Setup a separate thread that will be used to service ROS callbacks.
    // NOTE: We run the ROS loop in a separate thread as external calls such
    // as service callbacks to load controllers can block the (main) control loop
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    // Setup for the control loop.
    ros::Time prev_time = ros::Time::now();
    ros::Rate rate(10.0); // 10 Hz rate
    
    // Blocks until shutdown signal recieved
    while (ros::ok())
    {
        // Basic bookkeeping to get the system time in order to compute the control period.
        const ros::Time     time   = ros::Time::now();
        const ros::Duration period = time - prev_time;
        
        // Execution of the actual control loop.
        diffBot.read(time, period);
        // If needed, its possible to define transmissions in software by calling the 
        // transmission_interface::ActuatorToJointPositionInterface::propagate()
        // after reading the joint states.
        cm.update(time, period);
        // In case of software transmissions, use 
        // transmission_interface::JointToActuatorEffortHandle::propagate()
        // to convert from the joint space to the actuator space.
        diffBot.write(time, period);
        
        // All these steps keep getting repeated with the specified rate.
        rate.sleep();
    }
    return 0;
}
```

As we can see, the basic steps are to initialize the node, instantiate the hardware interface, pass it to a new controller manager and run the
control loop that does the following:

- Read joint states from the real robot hardware
- Update the `diff_drive_controller` with read values and compute the joint velocities using the target `cmd_vel`
- Write the computed values 

You may be wondering why the read values aren't returned from the `diffbot.read()` method and nothing is passed to the `diffbot.write()`.
This is because the `RobotHW::init()` method, shown in the first code snippet, is used to register the actuated joint names (described in the `diffbot_description`) to the `joint_position`, `joint_velocity` and `joint_effort` member variables of the custom robot hardware interface.
The class that registers the variables of the controller with the hardware interface member variables and thereby gives read access to all joint values 
without conflicting with other controllers, is the [`hardware_interface::JointStateInterface`](http://docs.ros.org/en/noetic/api/hardware_interface/html/c++/classhardware__interface_1_1JointStateInterface.html). 
ROS Control uses the `hardware_interface::VelocityJointInterface` (part of the [`joint_command_interface.h`](http://docs.ros.org/en/noetic/api/hardware_interface/html/c++/joint__command__interface_8h_source.html)) 
that registers the command member variable of the controller with the hardware interface to provide it the command that should be written to the actuators.

When the controller manager runs, the controllers will read from the `joint_position`, `joint_velocity` and `joint_effort` variables of the custom robot hardware interface, and the controller will write the desired command into the `joint_velocity_command` variable. It's mandatory to make sure the position, velocity and effort (effort is not needed in the case of the `diff_drive_controller`) variables always have the latest joint state available, and to make sure that whatever is written into the `joint_velocity_command` variable gets executed by the robot. As mentioned this can be done by implementing `hardware_interface::RobotHW::read()` and a `hardware_interface::RobotHW::write()` methods.

In the control loop the [overriden `hardware_interface::RobotHW::read()` method of DiffBot](https://github.com/fjp/diffbot/blob/522cba34117ea4cf90e3e0b5b9b70f0824e226fc/diffbot_base/src/diffbot_hw_interface.cpp#L106) is used to read the joint states. The `diff_drive_controller` works with a VelocityInterface which is why the `joint_position`, defined in rad, and `joint_velocity`, defined in rad/s, are calculated from the encoder ticks.


## PID Controller

Note the PID controller inside the hardware interface that is passed the error between velocity measured by the encoders and the target velocity computed
by the `diff_drive_controller`.

Note that the `diff_drive_controller` doesn't have a PID controller integrated, it doesn't take care if the wheels are actually turning.
As mentioned above, ROS Control expects that the commands sent by the controller are actually implemented on the real robot hardware and that the
joint states are always up to date. This means that the `diff_drive_controller` just uses the `twist_msg` on the `cmd_vel` topic for example from the `rqt_robot_steering` and converts it to a velocity command for the motors. It doesn't take the actual velocity of the motors into account. 
See [the code of `diff_drive_controller`](https://github.com/ros-controls/ros_controllers/blob/698f85b2c3467dfcc3ca5743d68deba03f3fcff2/diff_drive_controller/src/diff_drive_controller.cpp#L460) where the `joint_command_velocity` is calculated. 
{: .notice :}

This is why a PID controller is needed to avoid situations like the following where the robot moves not straigth although it is commanded to do so:



The PID used here inherits from the ROS Control [`control_toolbox::Pid`](http://wiki.ros.org/control_toolbox) that provides Dynamic Reconfigure out of the box to tune the proportional, integral and derivative gains. The behaviour when using only the P, I and D gains is that the output can overshoot and even change between positive and negative motor percent values because of a P gain that is too high. To avoid this a feed forward gain can help to reach the setpoint faster.
To add this feed forward gain to the dynamic reconfigure parameters it is necessary to add a new parameter configuration file in this package inside a `cfg` folder. 

For more details on ROS dynamic reconfigure see [the official tutorials](http://wiki.ros.org/dynamic_reconfigure/Tutorials).

After the use of the PID controller the robot is able to drive straight:

{% include video id="fdn5Mu0Qhl8" provider="youtube" %}


### Launch File

To run a single controller_manager, the one from the `diffbot_base` package defined inside `difbot_base.cpp` use the 
launch file from [`diffbot_base/launch/diffbot.launch`](https://github.com/fjp/diffbot/blob/master/ros/src/diffbot_base/launch/diffbot.launch):

```xml
<!-- https://github.com/ros-controls/ros_controllers/tree/kinetic-devel/diff_drive_controller/test -->
<launch>
    <!-- Load DiffBot model -->
    <param name="robot_description"
	   command="$(find xacro)/xacro '$(find diffbot_description)/urdf/diffbot.xacro'"/>

    <node name="diffbot_base" pkg="diffbot_base" type="diffbot_base"/>

    <!-- Load controller config to the parameter server -->
    <rosparam command="load" 
              file="$(find diffbot_control)/config/diffbot_control.yaml"/>

    <!-- Load the controllers -->
    <node name="controller_spawner" pkg="controller_manager" type="spawner" respawn="false"
        output="screen" ns="diffbot" args="joint_state_controller
                                            mobile_base_controller"/>
</launch>
```

This will load the DiffBot robot description onto the parameter server which is required  for the hardware interface that gets created inside the next
node `diffbot_base`. It creates the hardware interface and instantiates a new controller manager in the `diffbot_base.cpp`.
Finally the `spawner` from the `controller_manager` package is used to initialize and start the controllers defined in the `diffbot_control/config/diffbot_control.yaml`. The last step in this launch file is required to get the controllers initialized and started.
Another way would be to use `controller_manager::ControllerManager::loadControllers()` inside the `diffbot_base.cpp`.


After launching this launch file on DiffBot (Raspberry Pi) with 

```console
roslaunch diffbot_base diffbot.launch
```

the following parameters are stored on the parameter server:

```console
$ rosparam list
/diffbot/hardware_interface/joints
/diffbot/joint_state_controller/extra_joints
/diffbot/joint_state_controller/publish_rate
/diffbot/joint_state_controller/type
/diffbot/mobile_base_controller/base_frame_id
/diffbot/mobile_base_controller/left_wheel
/diffbot/mobile_base_controller/pose_covariance_diagonal
/diffbot/mobile_base_controller/publish_rate
/diffbot/mobile_base_controller/right_wheel
/diffbot/mobile_base_controller/twist_covariance_diagonal
/diffbot/mobile_base_controller/type
/diffbot/mobile_base_controller/wheel_radius
/diffbot/mobile_base_controller/wheel_separation
/robot_description
/rosdistro
/roslaunch/uris/host_tensorbook__46157
/roslaunch/uris/host_ubuntu__33729
/rosversion
/run_id
```

## Additional Requirements

Because the hardware interface subscribes to the encoders, that are connected to the Teensy MCU, and publishes to the motors via the motr driver node,
another launch will be required to run these additional nodes. See the `diffbot_bringup` package for this setup.

## Simulation


To have a simulation showing DiffBot, the second step is to use the [`diffbot_gazebo/launch/diffbot_base.launch`](https://github.com/fjp/diffbot/blob/master/ros/src/diffbot_gazebo/launch/diffbot_base.launch) on the work pc:

```console
$ roslaunch diffbot_gazebo diffbot_base.launch
```

This will launch the gazebo simulation, which can make use of the running controllers inside the controller manager too:

<figure>
    <a href="https://raw.githubusercontent.com/fjp/diffbot/master/docs/resources/ros_control_gazebo.png"><img src="https://raw.githubusercontent.com/fjp/diffbot/master/docs/resources/ros_control_gazebo.png"></a>
    <figcaption><a href="http://gazebosim.org/tutorials/?tut=ros_control" title="ROS Control with Gazebo">ROS Control with Gazebo</a> Overview.</figcaption>
</figure>


After launching the Gazebo simulation the controllers got uninitialized.
(It is assumed that the [`gazebo_ros_control`](https://github.com/ros-simulation/gazebo_ros_pkgs/tree/kinetic-devel/gazebo_ros_control) plugin that gets launched). Because of this the controllers have to be initialized and started again. For this the [`diffbot_base/launch/controllers.launch`](https://github.com/fjp/diffbot/blob/master/ros/src/diffbot_base/launch/controllers.launch) should be used.
This launch file is just loading and starting all controllers again. Note that using the `spawner` from the `controller_manager` package, like in the `diffbot_base/launch/diffbot.launch` results in an error. (TODO this needs some more testing).
