## Motor with Wheel Encoder

The [DG01D-E](https://www.sparkfun.com/products/16413) is a single hobby motor with a hall speed encoder. 
This motor requires a voltage between 3-9V, has a gearbox ratio of 1:48 and a speed of 90RPM at 4.5V. 
The voltage between positive and negative is determined according to the power supply voltage of the single chip microcomputer used, 
generally 3.3V or 5V is used.

The hall sensor can sense the North and South poles of its magnetic plate. 
When the hall sensor senses the South of the magnetic plate, the hall output will result in a high level. 
Meanwhile the North is the inverse and, when sensed, the hall output will result a low level.

### Terminal Pin Layout

The pins on the product are as follows, when looking at the connector on the housing, motor down/connector up, from right to left. The colors correspond to the included cable when plugged in to the connection slot.

- G (Blue): hall power negative
- H1 (Green): hall H1 output signal, square wave
- H2 (Yellow): hall H2 output signal, square wave
- V (Orange): hall power positive
- M+ (Red): motor positive pole
- M- (Brown): motor negative pole

The following image shows the motor from its side with the corresponding pin descriptions:

<figure>
    <a href="https://raw.githubusercontent.com/fjp/fjp.github.io/master/assets/collections/diffbot/components/dg01d-e-motor-with-encoder-pins.png"><img src="https://raw.githubusercontent.com/fjp/fjp.github.io/master/assets/collections/diffbot/components/dg01d-e-motor-with-encoder-pins.png"></a>
    <figcaption>DG01D-E Motor with encoder pin description.</figcaption>
</figure>


### Wheel Encoder Measurements

This section shows oscilloscope waveform measurements of the quadrature encoder in the DG01D-E motor. 
The motor is connected to the [Grove I2C Motor Driver](https://fjp.at/projects/diffbot/components/#control) that is powerd with 10 VDC. 
The `motor_example.py` applies 50-100% of the 10 VDC which leads to the following output voltages on the motor:

{% include video id="5GlZtQ87Co8" provider="youtube" %}

<details markdown="1"><summary>Voltage sweep measurements</summary>
- 0:00 Forward Speed 50: 6.5 VDC
- 0:12 Back Speed 50: 6.5 VDC 
- 0:23 Forward Speed 60: 6.9 VDC
- 0:34 Back Speed 60: 6.9 VDC
- 0:46 Forward Speed 70:  7.2 VDC
- 0:56 Back Speed 70:  7.2 VDC
- 1:07 Forward 80: 7.3 VDC
- 1:18 Back 80: 7.3 VDC
- 1:29 Forward 90: 7.6 VDC
- 1:41 Back 90: 7.6 VDC
- 1:52 Forward 100: 7.9 VDC
- 2:02 Back 100: 7.9 VDC
</details>


At the bottom of the pico scope window the cycle time, duty cycle, high and low pulse width measurements are shown for both encoder signals.
Oscilloscope is the [PicoScope 3000 Series](https://www.picotech.com/oscilloscope/3000/usb3-oscilloscope-logic-analyzer) with 2 Channels.
To download and install Pico Scope software on Linux refer to the [documentation](https://www.picotech.com/downloads/linux).

<details markdown="1"><summary>Summary of installation instructions</summary>

1. Add repository to the updater
```console
sudo bash -c 'echo "deb https://labs.picotech.com/debian/ picoscope main" >/etc/apt/sources.list.d/picoscope.list'
```
2. Import public key
```console
wget -O - https://labs.picotech.com/debian/dists/picoscope/Release.gpg.key | sudo apt-key add -
```
3. Update package manager cache
```console
sudo apt-get update
```

4. Install PicoScope

```console
sudo apt-get install picoscope
```

</details>


