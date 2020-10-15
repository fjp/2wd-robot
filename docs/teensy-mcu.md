## Teensy Setup

The Teensy 3.2 microcontroller (MCU) is used to get the ticks from the encoders attached to the motors and send this information (counts) as a message over the `/diffbot/ticks_left`
and `/diffbot/ticks_right` ropics. For this rosserial is running on the Teensy MCU which allows it to create a node on the Teensy that can communicate with
the ROS Master running on the Raspberry Pi.

To setup rosserial on the Raspberry Pi the following package has to be installed:

```
sudo apt install ros-noetic-rosserial
```
