## Teensy Setup

The Teensy 3.2 microcontroller (MCU) is used to get the ticks from the encoders attached to the motors and send this information (counts) as a message over the `/diffbot/ticks_left`
and `/diffbot/ticks_right` ropics. For this rosserial is running on the Teensy MCU which allows it to create a node on the Teensy that can communicate with
the ROS Master running on the Raspberry Pi.

To setup rosserial on the work PC and the Raspberry Pi the following package has to be installed:

```console
sudo apt install ros-noetic-rosserial
```

To program the Teensy board with the work PC the Arduino IDE with the Teensyduino add-on can be used. Other options are to use PlatformIO plugin for VSCode.
How to install the Arduino IDE and Teensyduino is listed in the [instructions on the Teensy website](https://www.pjrc.com/teensy/td_download.html).
Here the instructions to setup Teensyduino in Linux are listed:

> 1. Download the Linux udev rules (link at the top of this page) and copy the file to /etc/udev/rules.d.
>    `sudo cp 49-teensy.rules /etc/udev/rules.d/`
> 2. Download and extract one of Arduino's Linux packages.
>    Note: Arduino from Linux distro packages is not supported.
> 3. Download the corresponding Teensyduino installer.
> 4. Run the installer by adding execute permission and then execute it.
>    `chmod 755 TeensyduinoInstall.linux64`
>     `./TeensyduinoInstall.linux64`

The first step can be used on the work PC and the Raspberry Pi to enable the USB communication with the Teensy board.
Step two of these instructions are only necessary on the work PC to actually program the Teensy board.


TODO add teensy duino video


To check if the connection to the Teensy board works use these commands on the Raspberry Pi:

```console
$ lsusb
Bus 002 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
Bus 001 Device 003: ID 16c0:0483 Van Ooijen Technische Informatica Teensyduino Serial
Bus 001 Device 002: ID 2109:3431 VIA Labs, Inc. Hub
Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
```

And to see on which serial port it is connected use:

```console
$ ls /dev | grep ttyACM0
ttyACM0
```

If the output is empty it might be the case that the board is connected to another port like `ttyUSB0`.  

## Encoder Program

When installing Teensyduino new example programs are provided. One of them is to test Encoders. 
The code for the motor encoders uses it as basis together with a pubsub example from rosserial:

TODO link to code encoders.ino


After the program is flashed to the Teensy board it can be tested with the following procedure:

1. Create a rosserial node using `rosserial_python` package:

```console
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200
[INFO] [1602782359.573606]: ROS Serial Python Node
[INFO] [1602782359.600944]: Connecting to /dev/ttyACM0 at 115200 baud
[INFO] [1602782361.717904]: Requesting topics...
[ERROR] [1602782376.724880]: Unable to sync with device; possible link problem or link software version mismatch such as hydro rosserial_python with groovy Arduino

```

Each [DG01D-E](https://www.sparkfun.com/products/16413) motor encoder has two signal pins its built-in encoder. For these the Teensy pins 5, 6 are used for the left encoder and 7, 8 are used for the right one, see also the [Teensy pinout](https://www.pjrc.com/teensy/pinout.html).

TODO add teensy pinout image and connection schematic
