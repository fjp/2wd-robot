Part list and assembly of the robot platform and the components.

| Category     | Hardware          | Part Number                                       | Data Sheet & Info       |
|:------------:|:-----------------:|:-------------------------------------------------:|:-----------------------:|
| Accessories                                 |
|              | Case for Raspberry Pi 4 B | Slim acrylic case for Raspberry Pi 4, stackable, rainbow/transparent                          | [BerryBase](https://www.berrybase.de/en/new/slim-acrylic-case-for-raspberry-pi-4-stackable-rainbow) |
|              | Micro SD Card     | SanDisk 64GB Class 10                             | [SanDisk](), Ubuntu 18.04 Image |
|              | Robot Car Kit 2WD | [robot05](https://joy-it.net/en/products/robot05) | [Instructions manual](https://joy-it.net/files/files/Produkte/robot05/Robot05-Anleitung.pdf) |
|              | Power bank        | Intenso Powerbank S10000                          | [Intenso](https://www.intenso.de/en/products/powerbanks/powerbank-s10000) |
| Actuator                                                                                  |
|              | (Deprecated) Gearbox motor     | DC Gearbox motor - "TT Motor" - 200RPM - 3 to 6VDC | [Adafruit](https://www.adafruit.com/product/3777) |
|              | DG01E-E Motor with encoder | DG01E-E Hobby motor with quadrature encoder | [Sparkfun](https://www.sparkfun.com/products/16413) |
| Board                                                                                    |
|              | Raspberry Pi 4 B  | Raspberry Pi 4 B - 4 GB                           | [OEM Website](https://www.raspberrypi.org/products/raspberry-pi-4-model-b/) |
| Cables                                                                                   |
|             | Jumper - Female to Female |                                           |                         |
|              | Jumper - Male to Male     |                                           |                         |
|              | Micro USB - USB Cable     |                                           |                         |
|              | Camera extension cable    |                                           |                         |
|              | I2C 4 pin cable           |                                           |                         |
| Electronics                                                                                    |
|              | Fan                       | Fan 30x30x7mm 5V DC with Dupont connector | [BerryBase](https://www.berrybase.de/en/raspberry-pi-co/raspberry-pi/components/fan-30x30x7mm-5v-dc-with-dupont-connector) | 
|              | I2C motor driver          | Grove - I2C Motor Driver                  | [Seeed Studio](http://wiki.seeedstudio.com/Grove-I2C_Motor_Driver_V1.3/) |
|              | I2C Hub                   | Grove - I2C Hub                           | [Seeed Studio](http://wiki.seeedstudio.com/Grove-I2C_Hub/) |
| Human Machine Interface                   |
|              | OLED Display   | Grove OLED Display 0.96"                  | [Seeed Studio](http://wiki.seeedstudio.com/Grove-OLED_Display_0.96inch/) |
|              | LED Ring                  | NeoPixel Ring 12x5050 RGB LED             |  [Adafruit](https://www.adafruit.com/product/1643) |
| Sensors                                                                                   |
|              | Camera module             | Raspberry Pi - camera module v2.1         | [Raspberry Pi](https://www.raspberrypi.org/documentation/usage/camera/) |
|              | Ultrasonic ranger         | Grove - Ultrasonic Ranger                 | [Seeed Studio](http://wiki.seeedstudio.com/Grove-Ultrasonic_Ranger/) | 
|              | IMU                       | Adafruit 9-DOF Absolute Orientation IMU Fusion Breakout - BNO055 | [Adafruit](https://www.adafruit.com/product/2472) |
|              | Odometry                  | Joy-IT - LM393 Speed Sensor with H206 slot-type opto interrupter | [Joy-IT](https://joy-it.net/en/products/SEN-Speed) |

Order list

| Part                    | Store |
|:------------------------|:---------------------------------------------------------------------------:|
| Raspberry Pi 4 B (4 Gb) | [Amazon.com](https://amzn.to/3ltuJUo), [Amazon.de](https://amzn.to/2IchIAc) |
| SanDisk 64 GB SD Card Class 10 | [Amazon.com](https://amzn.to/2GLOyr0), [Amazon.de](https://amzn.to/3dcFmYE) |
|Robot Smart Chassis Kit  | [Amazon.com](https://amzn.to/34GXNAK), [Amazon.de](https://amzn.to/2Gy3CJ4) |
| SLAMTEC RPLidar A2M8 (12 m) | [Amazon.com](https://amzn.to/3lthTFz), [Amazon.de](https://amzn.to/30MyImR) |
| Grove Ultrasonic Ranger | [Amazon.com](https://amzn.to/36M9TLS), [Amazon.de](https://amzn.to/34GZmyC) |
| Raspi Camera Module V2, 8 MP, 1080p | [Amazon.com](https://amzn.to/2Ib9fgG), [Amazon.de](https://amzn.to/2FdVDQF) |
| Grove Motor Driver | [seeedstudio.com](https://www.seeedstudio.com/Grove-I2C-Motor-Driver-with-L298.html), [Amazon.de](https://amzn.to/36M8O6M) |
| I2C Hub | [seeedstudio.com](https://www.seeedstudio.com/Grove-I2C-Hub.html), [Amazon.de](https://amzn.to/34CGEbz) |


Additional (Optional) Equipment

| Part                                   | Store |
|:---------------------------------------|:------------------------------------:|
| PicoScope 3000 Series Oscilloscope 2CH | [Amazon.de](https://amzn.to/33I5tUb) |
| VOLTCRAFT PPS-16005                    | [Amazon.de](https://amzn.to/3iKsI4a) |

## Board - Raspberry Pi 4 B

The main processing unit of the robot is a [Raspberry Pi 4 B](https://www.raspberrypi.org/products/raspberry-pi-4-model-b/) 
with 4 GB of RAM. 

<figure class="half">
    <a href="/assets/collections/diffbot/components/raspberry-pi-4.jpg"><img src="/assets/collections/diffbot/components/raspberry-pi-4.jpg"></a>
    <a href="/assets/collections/diffbot/components/raspberry-pi-4-ports.jpg"><img src="/assets/collections/diffbot/components/raspberry-pi-4-ports.jpg"></a>
    <figcaption>Raspberry Pi 4 B - 4 GB RAM variant.</figcaption>
</figure>

## Accessories and Electronics

### Case and Cooling

To protect the Rasbperry Pi 4 B we choose a case that provides access to all its ports.
The following images show a stackable acrylic case in rainbow colors.

<figure class="half">
    <a href="/assets/collections/diffbot/components/case_side_raspberry4_rainbow.jpg"><img src="/assets/collections/diffbot/components/case_side_raspberry4_rainbow.jpg"></a>
    <a href="/assets/collections/diffbot/components/case_bottom_raspberry4_rainbow.jpg"><img src="/assets/collections/diffbot/components/case_bottom_raspberry4_rainbow.jpg"></a>
    <figcaption>Stackable Rainbow Case for Raspberry Pi 4 B.</figcaption>
</figure>

With this case it is possible to install four heatsinks and apply a fan as cooling equipment for the electronics of the Raspberry Pi 4 B such as its ARM processor.

<figure class="half">
    <a href="/assets/collections/diffbot/components/heatsink.jpg"><img src="/assets/collections/diffbot/components/heatsink.jpg"></a>
    <a href="/assets/collections/diffbot/components/case_fan_side_raspberry4_rainbow.jpg"><img src="/assets/collections/diffbot/components/case_fan_side_raspberry4_rainbow.jpg"></a>
    <figcaption>Heatsinks and cooling fan for Raspberry Pi 4 B.</figcaption>
</figure>


### SD Card

The Raspberry Pi requires a medium to boot from. 
For this we will use a micro sd card because it is lightweight and easy to flash new operating systems. 

<figure>
    <a href="/assets/collections/diffbot/components/sdcard.jpg"><img src="/assets/collections/diffbot/components/sdcard.jpg"></a>
    <figcaption>SanDisk Micro SD Card Class 10.</figcaption>
</figure>

Although a micro sd card won't last that long compared to an hard disk drive (HDD) or solid state disk (SSD) it is well suited for testing. Because sd cards are slower when reading and writing data you should make sure to choose a micro sd card with high 
performance ratings. For the Raspberry Pi a Class 10 micro sd card is recommended. 
Regarding speed, the Pi has a limited bus speed of approximately 20 MB/s ([source](https://raspberrypi.stackexchange.com/questions/43618/raspberry-pi-3-micro-sd-card-speed))

### Robot Base

The [Robot Car Kit 2WD](https://joy-it.net/en/products/robot05) from Joy-IT (article no.: robot05) is used as the base for the autonomous mobile robot. 

<figure>
    <a href="/assets/collections/diffbot/car-kit05.jpg"><img src="/assets/collections/diffbot/car-kit05.jpg"></a>
    <figcaption>Parts of the 2WD Robot Car Kit 05 from Joy-IT.</figcaption>
</figure>

The acrylic chassis has many holes which allow to mount a mounting plate that can hold different development boards.
It allows also to mount a Raspberry Pi 4 B, which will be used in this project. Two wheels, hence 2WD, are included in the kit which can be attached to the motors that are provided too. A third caster wheel is provided which allows the robot to spin on the spot. This means the robot can be described by a holonomic model. 

The motors operate in a range between 3 to 6 Volts DC and make it possible to mount a punched disk for speed measurements. 
With that punched disk and additional speed sensors it is possible to implement odometry in ROS. 
To power the motors a battery compartment is available together with a switch to turn the robot on or off.

### Power Supplies

As mentioned the robot will be equipped with a 5V/2.1A USB-C powerbank to supply the Raspberry Pi 4 B with 5 V.

<figure class="half">
    <a href="/assets/collections/diffbot/components/powerbank_top.jpg"><img src="/assets/collections/diffbot/components/powerbank_top.jpg"></a>
    <a href="/assets/collections/diffbot/components/powerbank_bottom.jpg"><img src="/assets/collections/diffbot/components/powerbank_bottom.jpg"></a>
    <figcaption>Power bank with 10.000 mAh from Intenso.</figcaption>
</figure>

To power the motors the provided battery compartment will be used, which holds four AA batteries $4 \cdot 1.5\text{V} = 6\text{V}$.

### I2C Hub

The Raspberry Pi provides just two I2C ports, which is why we will use a I2C hub. With the four port I2C hub from Grove it is possible to connect three I2C devices to a single I2C port of the Raspberry Pi

<figure class="half">
    <a href="/assets/collections/diffbot/components/i2c-hub-front.jpg"><img src="/assets/collections/diffbot/components/i2c-hub-front.jpg"></a>
    <a href="/assets/collections/diffbot/components/i2c-hub-back.jpg"><img src="/assets/collections/diffbot/components/i2c-hub-back.jpg"></a>
    <figcaption>Grove I2C Hub.</figcaption>
</figure>

### Breadboard and GPIO Extension Cable

Optional but helpful for testing is a breadboard and a GPIO extension cable suitable for the Raspberry Pi 4 B.

<figure>
    <a href="/assets/collections/diffbot/components/bread-board-gpio-extension.jpg"><img src="/assets/collections/diffbot/components/bread-board-gpio-extension.jpg"></a>
    <figcaption>Breadboard with GPIO extension cable.</figcaption>
</figure>

## Sensors

Sensors are used to sense the environment and to collect information of the current state.
For this 2WD robot the sensors are categorized into perception and localization which are explained in the following two sections.

### Perception

Perception sensors of the 2WD robot will be used to avoid collisions using ultrasonic rangers.
Another use case is to detect and follow objects using a camera.

#### Ultrasonic Ranger

To avoid obstacles the robot will carry a [Grove - Ultrasonic Ranger](http://wiki.seeedstudio.com/Grove-Ultrasonic_Ranger/)
at the front. 

<figure class="half">
    <a href="/assets/collections/diffbot/components/ultrasonic-side.jpg"><img src="/assets/collections/diffbot/components/ultrasonic-side.jpg"></a>
    <a href="/assets/collections/diffbot/components/ultrasonic-front.jpg"><img src="/assets/collections/diffbot/components/ultrasonic-front.jpg"></a>
    <figcaption>Grove Ultrasonic Ranger for obstacle avoidance.</figcaption>
</figure>

It is a non-contact distance measurement module which works at 40KHz and can be interfaced via a single [GPIO](https://www.raspberrypi.org/documentation/usage/gpio/). For example [physical pin 11](https://pinout.xyz/pinout/pin11_gpio17) of the Raspberry Pi connected to the `SIG` pin on the sensor can provide the PWM communication.

|Parameter|	Value/Range|
|:------|:------------------|
|Operating voltage|	3.2~5.2V|
|Operating current|	8mA|
|Ultrasonic frequency|	40kHz|
|Measuring range|	2-350cm|
|Resolution|	1cm|
|Output|PWM|
|Size|50mm X 25mm X 16mm|
|Weight|13g|
|Measurement angle|15 degree|
|Working temperature|-10~60 degree C|
|Trigger signal|10uS TTL|
|Echo signal|TTL|

The code that will be used to wrap this sensor as a ROS node can be found in the [Grove Raspberry Pi](https://github.com/Seeed-Studio/Grove-RaspberryPi/blob/master/Grove%20-%20Ultrasonic%20Ranger/ultrasonic.py) repository on GitHub.

As an alternative we could use the [HC SR04](https://www.seeedstudio.com/blog/2019/11/04/hc-sr04-features-arduino-raspberrypi-guide/).

#### Camera

<figure>
    <a href="/assets/collections/diffbot/components/rpi-camera.jpg"><img src="/assets/collections/diffbot/components/rpi-camera.jpg"></a>
    <figcaption>RPi Camera v2.</figcaption>
</figure>


### Localization

#### Inertial Measurement Unit

An intertial measurement unit (IMU) measures the acceleration and orientation through gyroscopes directly.
Other states such as the velocity can then be calculated.
For this the [Adafruit 9-DOF Absolute Orientation IMU Fusion Breakout - BNO055]((https://www.adafruit.com/product/2472)) is used.

<figure>
    <a href="/assets/collections/diffbot/components/bno055.jpg"><img src="/assets/collections/diffbot/components/bno055.jpg"></a>
    <figcaption>9-DOF Absolute Orientation IMU Fusion Breakout - BNO055 from Adafruit.</figcaption>
</figure>

#### Odometry

For the used odometry sensor see the section below [Motor and Wheel Encoder](/projects/diffbot/components/#motor-and-wheel-encoder)

<details><summary>Alternative Optical Sensor</summary>

To estimate the change in position over time ([odometry](https://en.wikipedia.org/wiki/Odometry)) the robot will
utilize an [optical speed sensor](https://en.wikipedia.org/wiki/Wheel_speed_sensor#Optical_sensor). 
Specifically the [Joy-IT Speed Sensor](https://joy-it.net/en/products/SEN-Speed) which combines a LM393 ([datasheet](http://www.ti.com/lit/ds/symlink/lm2903-n.pdf)) [comperator](https://en.wikipedia.org/wiki/Comparator) and a H206 slot-type opto interrupter.

<figure class="half">
    <a href="/assets/collections/diffbot/components/speed-sensor-front.jpg"><img src="/assets/collections/diffbot/components/speed-sensor-front.jpg"></a>
    <a href="/assets/collections/diffbot/components/speed-sensor-back.jpg"><img src="/assets/collections/diffbot/components/speed-sensor-back.jpg"></a>
    <figcaption>LM393 Speed Sensor from Joy-IT.</figcaption>
</figure>

Technical Specifications:

- Dimensions: 32 x 14 x 7mm 
- Operating voltage: 3.3V to 5V (we will use 3.3V)
- Two outputs: Digital (D0) and Analog (A0)


References:
https://dronebotworkshop.com/robot-car-with-speed-sensors/

</details>

## Actuators

- [Grove - I2C Motor Driver V1.3](http://wiki.seeedstudio.com/Grove-I2C_Motor_Driver_V1.3/)

### Control

To drive the two motors of the car kit we use the 
[Grove - I2C Motor Driver V1.3](http://wiki.seeedstudio.com/Grove-I2C_Motor_Driver_V1.3/) from Seeed Studio.

<figure>
    <a href="/assets/collections/diffbot/components/motor-driver.jpg"><img src="/assets/collections/diffbot/components/motor-driver.jpg"></a>
    <figcaption>Grove - I2C Motor Driver.</figcaption>
</figure>


### Motor and Wheel Encoder

The [DG01E-E Hobby Motor](https://www.sparkfun.com/products/16413) has a quadrature encoder built in, 
which makes it easy to assemble the robot and saves space because of no additional
(optical or magnetic) wheel encoders.

<figure class="half">
    <a href="/assets/collections/diffbot/components/dg01d-e-motor-with-encoder.jpg"><img src="/assets/collections/diffbot/components/dg01d-e-motor-with-encoder.jpg"></a>
    <a href="/assets/collections/diffbot/components/dg01d-e-motor-with-encoder-pins.png"><img src="/assets/collections/diffbot/components/dg01d-e-motor-with-encoder-pins.png"></a>
    <figcaption>DG01D-E Motor with wheel encoders.</figcaption>
</figure>

<details><summary>Alternative Brushed Gear Motor</summary>

### Brushed Gearbox Motor

<figure>
    <a href="/assets/collections/diffbot/components/gearbox-motor-close.jpg"><img src="/assets/collections/diffbot/components/gearbox-motor-close.jpg"></a>
    <figcaption>DC Gearbox motor - "TT Motor" - 200RPM - 3 to 6VDC.</figcaption>
</figure>

</details>

## Human Machine Interface (HMI)

The human machine interface is the layer between the user and the robot. 

### OLED Display

To update the user with status messages the robot has a 0.96 inch oled (organic ligth emitting diode) display.
The oled display used is the [Grove I2C 0.96 inch OLED display](http://wiki.seeedstudio.com/Grove-OLED_Display_0.96inch/) 
from Seeed Studio.

<figure class="third">
    <a href="/assets/collections/diffbot/components/oled-01.jpg"><img src="/assets/collections/diffbot/components/oled-01.jpg"></a>
    <a href="/assets/collections/diffbot/components/oled-02.jpg"><img src="/assets/collections/diffbot/components/oled-02.jpg"></a>
    <a href="/assets/collections/diffbot/components/oled-03.jpg"><img src="/assets/collections/diffbot/components/oled-03.jpg"></a>
    <figcaption>Grove - I2C 0.96 inch OLED Display.</figcaption>
</figure>

The display is connected to the RPi via I2C on the physical pins 27 (scl) and 28 (sda), refere to the [pinout](https://pinout.xyz/pinout/i2c).

[Library](https://github.com/DexterInd/GrovePi/blob/master/Software/Python/grove_i2c_oled_128_64/grove_128_64_oled.py)