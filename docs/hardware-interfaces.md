The hardware interfaces provide an interface between the components (sensors and actuators) of the 2WD robot and
the main processing unit, the Raspberry Pi 4 B.

## GPIO

Currently, three GPIO pins are used to connect the ultrasonic ranger and two speed sensors.

The ultrasonic ranger uses just a single GPIO pin for communicating its measured distances.
Therefore, we can use one of the GPIO pins such as [physical pin 11](https://pinout.xyz/pinout/pin11_gpio17).

The LM393 speed sensors also use a single digital GPIO pin each. These pins will be setup using software interrupts with the [RPi.GPIO](https://pypi.org/project/RPi.GPIO/) library. 

## Prepare I2C Connection

The I2C connections are used for multiple components such as the motor driver and the oled display.
Using these ports on the Raspberry Pi 4 B, requires that we enable the I2C interface. 

To do so, we will use the tool `i2cdetect` which requires that we install a tool on Ubuntu called `i2c-tools`:

```bash
fjp@ubuntu:~/git/2wd-robot$ i2cdetect

Command 'i2cdetect' not found, but can be installed with:

sudo apt install i2c-tools

fjp@ubuntu:~/git/2wd-robot$ sudo apt install i2c-tools
```

To test if the i2c ports are working we use the following commands:

```bash
$ i2cdetect -y 0
Error: Could not open file '/dev/i2c-0' or '/dev/i2c/0': No such file or directory
$ i2cdetect -y 1
Error: Could not open file '/dev/i2c-1' or '/dev/i2c/1': No such file or directory

The ports are not setup correctly yet, which is why we need to enable the following two lines in the `/boot/firmware/config.txt` file:

```bash
dtparam=i2c0=on
dtparam=i2c1=on
```

<figure>
    <a href="https://github.com/fjp/2wd-robot/raw/master/docs/i2c-rpi-pinout.png"><img src="https://github.com/fjp/2wd-robot/raw/master/docs/i2c-rpi-pinout.png"></a>
    <figcaption><a href="https://pinout.xyz/pinout/i2c" title="I2C Pinout">I2C Pinout</a> on Raspberry Pi 4 B.</figcaption>
</figure>


On Raspian Buster, the official Raspberry OS, we could use the `raspi-config` tool:

<pre><font color="#8AE234"><b>fjp@ubuntu</b></font>:<font color="#729FCF"><b>~/git/2wd-robot</b></font>$ sudo raspi-config</pre>

<pre><span style="background-color:#75507B"><font color="#D3D7CF">Raspberry Pi 4 Model B Rev 1.1</font></span>



<span style="background-color:#D3D7CF"><font color="#2E3436">┌──────────────────┤ </font></span><span style="background-color:#D3D7CF"><font color="#CC0000">Raspberry Pi Software Configuration Tool (raspi-config)</font></span><span style="background-color:#D3D7CF"><font color="#2E3436"> ├───────────────────┐</font></span>
<span style="background-color:#D3D7CF"><font color="#2E3436">│                                                                                                │</font></span>
<span style="background-color:#D3D7CF"><font color="#2E3436">│      1 Change User Password Change password for the current user                               │</font></span>
<span style="background-color:#D3D7CF"><font color="#2E3436">│      2 Network Options      Configure network settings                                         │</font></span>
<span style="background-color:#D3D7CF"><font color="#2E3436">│      3 Boot Options         Configure options for start-up                                     │</font></span>
<span style="background-color:#D3D7CF"><font color="#2E3436">│      4 Localisation Options Set up language and regional settings to match your location       │</font></span>
<span style="background-color:#D3D7CF"><font color="#2E3436">│      </font></span><span style="background-color:#CC0000"><font color="#D3D7CF">5 Interfacing Options  Configure connections to peripherals                        </font></span><span style="background-color:#D3D7CF"><font color="#2E3436">       │</font></span>
<span style="background-color:#D3D7CF"><font color="#2E3436">│      6 Overclock            Configure overclocking for your Pi                                 │</font></span>
<span style="background-color:#D3D7CF"><font color="#2E3436">│      7 Advanced Options     Configure advanced settings                                        │</font></span>
<span style="background-color:#D3D7CF"><font color="#2E3436">│      8 Update               Update this tool to the latest version                             │</font></span>
<span style="background-color:#D3D7CF"><font color="#2E3436">│      9 About raspi-config   Information about this configuration tool                          │</font></span>
<span style="background-color:#D3D7CF"><font color="#2E3436">│                                                                                                │</font></span>
<span style="background-color:#D3D7CF"><font color="#2E3436">│                                                                                                │</font></span>
<span style="background-color:#D3D7CF"><font color="#2E3436">│                                                                                                │</font></span>
<span style="background-color:#D3D7CF"><font color="#2E3436">│                           &lt;Select&gt;                           &lt;Finish&gt;                          │</font></span>
<span style="background-color:#D3D7CF"><font color="#2E3436">│                                                                                                │</font></span>
<span style="background-color:#D3D7CF"><font color="#2E3436">└────────────────────────────────────────────────────────────────────────────────────────────────┘</font></span>


</pre>

Select the i2c option:

<pre>



<span style="background-color:#D3D7CF"><font color="#2E3436">┌──────────────────┤ </font></span><span style="background-color:#D3D7CF"><font color="#CC0000">Raspberry Pi Software Configuration Tool (raspi-config)</font></span><span style="background-color:#D3D7CF"><font color="#2E3436"> ├───────────────────┐</font></span>
<span style="background-color:#D3D7CF"><font color="#2E3436">│                                                                                                │</font></span>
<span style="background-color:#D3D7CF"><font color="#2E3436">│        P1 Camera      Enable/Disable connection to the Raspberry Pi Camera                     │</font></span>
<span style="background-color:#D3D7CF"><font color="#2E3436">│        P2 SSH         Enable/Disable remote command line access to your Pi using SSH           │</font></span>
<span style="background-color:#D3D7CF"><font color="#2E3436">│        P3 VNC         Enable/Disable graphical remote access to your Pi using RealVNC          │</font></span>
<span style="background-color:#D3D7CF"><font color="#2E3436">│        P4 SPI         Enable/Disable automatic loading of SPI kernel module                    │</font></span>
<span style="background-color:#D3D7CF"><font color="#2E3436">│        </font></span><span style="background-color:#CC0000"><font color="#D3D7CF">P5 I2C         Enable/Disable automatic loading of I2C kernel module            </font></span><span style="background-color:#D3D7CF"><font color="#2E3436">        │</font></span>
<span style="background-color:#D3D7CF"><font color="#2E3436">│        P6 Serial      Enable/Disable shell and kernel messages on the serial connection        │</font></span>
<span style="background-color:#D3D7CF"><font color="#2E3436">│        P7 1-Wire      Enable/Disable one-wire interface                                        │</font></span>
<span style="background-color:#D3D7CF"><font color="#2E3436">│        P8 Remote GPIO Enable/Disable remote access to GPIO pins                                │</font></span>
<span style="background-color:#D3D7CF"><font color="#2E3436">│                                                                                                │</font></span>
<span style="background-color:#D3D7CF"><font color="#2E3436">│                                                                                                │</font></span>
<span style="background-color:#D3D7CF"><font color="#2E3436">│                                                                                                │</font></span>
<span style="background-color:#D3D7CF"><font color="#2E3436">│                                                                                                │</font></span>
<span style="background-color:#D3D7CF"><font color="#2E3436">│                           &lt;Select&gt;                           &lt;Back&gt;                            │</font></span>
<span style="background-color:#D3D7CF"><font color="#2E3436">│                                                                                                │</font></span>
<span style="background-color:#D3D7CF"><font color="#2E3436">└────────────────────────────────────────────────────────────────────────────────────────────────┘</font></span>


</pre>

And enable the interface:

<pre>


<span style="background-color:#75507B"><font color="#EEEEEC">                   </font></span><span style="background-color:#D3D7CF"><font color="#2E3436">┌──────────────────────────────────────────────────────────┐</font></span>
<span style="background-color:#75507B"><font color="#EEEEEC">                   </font></span><span style="background-color:#D3D7CF"><font color="#2E3436">│                                                          │</font></span><span style="background-color:#2E3436"><font color="#EEEEEC"> </font></span>
<span style="background-color:#75507B"><font color="#EEEEEC">                   </font></span><span style="background-color:#D3D7CF"><font color="#2E3436">│ Would you like the ARM I2C interface to be enabled?      │</font></span><span style="background-color:#2E3436"><font color="#EEEEEC"> </font></span>
<span style="background-color:#75507B"><font color="#EEEEEC">                   </font></span><span style="background-color:#D3D7CF"><font color="#2E3436">│                                                          │</font></span><span style="background-color:#2E3436"><font color="#EEEEEC"> </font></span>
<span style="background-color:#75507B"><font color="#EEEEEC">                   </font></span><span style="background-color:#D3D7CF"><font color="#2E3436">│                                                          │</font></span><span style="background-color:#2E3436"><font color="#EEEEEC"> </font></span>
<span style="background-color:#75507B"><font color="#EEEEEC">                   </font></span><span style="background-color:#D3D7CF"><font color="#2E3436">│                                                          │</font></span><span style="background-color:#2E3436"><font color="#EEEEEC"> </font></span>
<span style="background-color:#75507B"><font color="#EEEEEC">                   </font></span><span style="background-color:#D3D7CF"><font color="#2E3436">│                                                          │</font></span><span style="background-color:#2E3436"><font color="#EEEEEC"> </font></span>
<span style="background-color:#75507B"><font color="#EEEEEC">                   </font></span><span style="background-color:#D3D7CF"><font color="#2E3436">│                                                          │</font></span><span style="background-color:#2E3436"><font color="#EEEEEC"> </font></span>
<span style="background-color:#75507B"><font color="#EEEEEC">                   </font></span><span style="background-color:#D3D7CF"><font color="#2E3436">│                                                          │</font></span><span style="background-color:#2E3436"><font color="#EEEEEC"> </font></span>
<span style="background-color:#75507B"><font color="#EEEEEC">                   </font></span><span style="background-color:#D3D7CF"><font color="#2E3436">│                                                          │</font></span><span style="background-color:#2E3436"><font color="#EEEEEC"> </font></span>
<span style="background-color:#75507B"><font color="#EEEEEC">                   </font></span><span style="background-color:#D3D7CF"><font color="#2E3436">│                                                          │</font></span><span style="background-color:#2E3436"><font color="#EEEEEC"> </font></span>
<span style="background-color:#75507B"><font color="#EEEEEC">                   </font></span><span style="background-color:#D3D7CF"><font color="#2E3436">│                                                          │</font></span><span style="background-color:#2E3436"><font color="#EEEEEC"> </font></span>
<span style="background-color:#75507B"><font color="#EEEEEC">                   </font></span><span style="background-color:#D3D7CF"><font color="#2E3436">│                                                          │</font></span><span style="background-color:#2E3436"><font color="#EEEEEC"> </font></span>
<span style="background-color:#75507B"><font color="#EEEEEC">                   </font></span><span style="background-color:#D3D7CF"><font color="#2E3436">│                                                          │</font></span><span style="background-color:#2E3436"><font color="#EEEEEC"> </font></span>
<span style="background-color:#75507B"><font color="#EEEEEC">                   </font></span><span style="background-color:#D3D7CF"><font color="#2E3436">│                                                          │</font></span><span style="background-color:#2E3436"><font color="#EEEEEC"> </font></span>
<span style="background-color:#75507B"><font color="#EEEEEC">                   </font></span><span style="background-color:#D3D7CF"><font color="#2E3436">│                                                          │</font></span><span style="background-color:#2E3436"><font color="#EEEEEC"> </font></span>
<span style="background-color:#75507B"><font color="#EEEEEC">                   </font></span><span style="background-color:#D3D7CF"><font color="#2E3436">│                                                          │</font></span><span style="background-color:#2E3436"><font color="#EEEEEC"> </font></span>
<span style="background-color:#75507B"><font color="#EEEEEC">                   </font></span><span style="background-color:#D3D7CF"><font color="#2E3436">│               </font></span><span style="background-color:#CC0000"><font color="#D3D7CF">&lt;Yes&gt;</font></span><span style="background-color:#D3D7CF"><font color="#2E3436">                  &lt;No&gt;                │</font></span><span style="background-color:#2E3436"><font color="#EEEEEC"> </font></span>
<span style="background-color:#75507B"><font color="#EEEEEC">                   </font></span><span style="background-color:#D3D7CF"><font color="#2E3436">│                                                          │</font></span><span style="background-color:#2E3436"><font color="#EEEEEC"> </font></span>
<span style="background-color:#75507B"><font color="#EEEEEC">                   </font></span><span style="background-color:#D3D7CF"><font color="#2E3436">└──────────────────────────────────────────────────────────┘</font></span><span style="background-color:#2E3436"><font color="#EEEEEC"> </font></span>
<span style="background-color:#75507B"><font color="#EEEEEC">                    </font></span><span style="background-color:#2E3436"><font color="#EEEEEC">                                                            </font></span>

</pre>

Confirm the activation and restart the RPi:

<pre>


<span style="background-color:#75507B"><font color="#EEEEEC">                   </font></span><span style="background-color:#D3D7CF"><font color="#2E3436">┌──────────────────────────────────────────────────────────┐</font></span>
<span style="background-color:#75507B"><font color="#EEEEEC">                   </font></span><span style="background-color:#D3D7CF"><font color="#2E3436">│                                                          │</font></span><span style="background-color:#2E3436"><font color="#EEEEEC"> </font></span>
<span style="background-color:#75507B"><font color="#EEEEEC">                   </font></span><span style="background-color:#D3D7CF"><font color="#2E3436">│ The ARM I2C interface is enabled                         │</font></span><span style="background-color:#2E3436"><font color="#EEEEEC"> </font></span>
<span style="background-color:#75507B"><font color="#EEEEEC">                   </font></span><span style="background-color:#D3D7CF"><font color="#2E3436">│                                                          │</font></span><span style="background-color:#2E3436"><font color="#EEEEEC"> </font></span>
<span style="background-color:#75507B"><font color="#EEEEEC">                   </font></span><span style="background-color:#D3D7CF"><font color="#2E3436">│                                                          │</font></span><span style="background-color:#2E3436"><font color="#EEEEEC"> </font></span>
<span style="background-color:#75507B"><font color="#EEEEEC">                   </font></span><span style="background-color:#D3D7CF"><font color="#2E3436">│                                                          │</font></span><span style="background-color:#2E3436"><font color="#EEEEEC"> </font></span>
<span style="background-color:#75507B"><font color="#EEEEEC">                   </font></span><span style="background-color:#D3D7CF"><font color="#2E3436">│                                                          │</font></span><span style="background-color:#2E3436"><font color="#EEEEEC"> </font></span>
<span style="background-color:#75507B"><font color="#EEEEEC">                   </font></span><span style="background-color:#D3D7CF"><font color="#2E3436">│                                                          │</font></span><span style="background-color:#2E3436"><font color="#EEEEEC"> </font></span>
<span style="background-color:#75507B"><font color="#EEEEEC">                   </font></span><span style="background-color:#D3D7CF"><font color="#2E3436">│                                                          │</font></span><span style="background-color:#2E3436"><font color="#EEEEEC"> </font></span>
<span style="background-color:#75507B"><font color="#EEEEEC">                   </font></span><span style="background-color:#D3D7CF"><font color="#2E3436">│                                                          │</font></span><span style="background-color:#2E3436"><font color="#EEEEEC"> </font></span>
<span style="background-color:#75507B"><font color="#EEEEEC">                   </font></span><span style="background-color:#D3D7CF"><font color="#2E3436">│                                                          │</font></span><span style="background-color:#2E3436"><font color="#EEEEEC"> </font></span>
<span style="background-color:#75507B"><font color="#EEEEEC">                   </font></span><span style="background-color:#D3D7CF"><font color="#2E3436">│                                                          │</font></span><span style="background-color:#2E3436"><font color="#EEEEEC"> </font></span>
<span style="background-color:#75507B"><font color="#EEEEEC">                   </font></span><span style="background-color:#D3D7CF"><font color="#2E3436">│                                                          │</font></span><span style="background-color:#2E3436"><font color="#EEEEEC"> </font></span>
<span style="background-color:#75507B"><font color="#EEEEEC">                   </font></span><span style="background-color:#D3D7CF"><font color="#2E3436">│                                                          │</font></span><span style="background-color:#2E3436"><font color="#EEEEEC"> </font></span>
<span style="background-color:#75507B"><font color="#EEEEEC">                   </font></span><span style="background-color:#D3D7CF"><font color="#2E3436">│                                                          │</font></span><span style="background-color:#2E3436"><font color="#EEEEEC"> </font></span>
<span style="background-color:#75507B"><font color="#EEEEEC">                   </font></span><span style="background-color:#D3D7CF"><font color="#2E3436">│                                                          │</font></span><span style="background-color:#2E3436"><font color="#EEEEEC"> </font></span>
<span style="background-color:#75507B"><font color="#EEEEEC">                   </font></span><span style="background-color:#D3D7CF"><font color="#2E3436">│                                                          │</font></span><span style="background-color:#2E3436"><font color="#EEEEEC"> </font></span>
<span style="background-color:#75507B"><font color="#EEEEEC">                   </font></span><span style="background-color:#D3D7CF"><font color="#2E3436">│                          </font></span><span style="background-color:#CC0000"><font color="#D3D7CF">&lt;Ok&gt;</font></span><span style="background-color:#D3D7CF"><font color="#2E3436">                            │</font></span><span style="background-color:#2E3436"><font color="#EEEEEC"> </font></span>
<span style="background-color:#75507B"><font color="#EEEEEC">                   </font></span><span style="background-color:#D3D7CF"><font color="#2E3436">│                                                          │</font></span><span style="background-color:#2E3436"><font color="#EEEEEC"> </font></span>
<span style="background-color:#75507B"><font color="#EEEEEC">                   </font></span><span style="background-color:#D3D7CF"><font color="#2E3436">└──────────────────────────────────────────────────────────┘</font></span><span style="background-color:#2E3436"><font color="#EEEEEC"> </font></span>
<span style="background-color:#75507B"><font color="#EEEEEC">                    </font></span><span style="background-color:#2E3436"><font color="#EEEEEC">                                                            </font></span>

</pre>
