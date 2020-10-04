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


### Wheel Encoder Measurements

{% include video id="31Zix56L-IA" provider="youtube" %}
