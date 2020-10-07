## Power Supply

DiffBot uses two different power sources. One USB type C power bank providing 5V to the Raspberry Pi 4B.
The other source comes from battery holder, mounted at the bottom of the base plate which can hold up to four AA sized batteries.
The standard [AA battery](https://en.wikipedia.org/wiki/AA_battery) has a charge of 2700 mAh and a nominal
Voltage of 1.5 V. This means that with four batteries in series the total Voltage supplied to the motor driver would
be 6 V. The voltage drop on the motor driver is roughly 2.12 V which means that each motor would operate on 3.88 V.
According to the [datasheet](https://cdn.sparkfun.com/assets/8/3/b/e/4/DS-16413-DG01D-E_Motor_with_Encoder.pdf) the 
[DG01D-E motor](https://www.sparkfun.com/products/16413) requires a voltage between 3-9 V, has a gearbox ratio of 1:48 and a speed of 90RPM at 4.5V.
Therefore 3.88 V is at the lower operating range of this motor, hence a higher input source to the motor driver is requried to provide
higher output voltage to the motors. The maximum allowed input Voltage to the motor driver is 15 V, hence a power source between 6 V to 15 V should be used,
in the range of 10 V to 15 V to drive with different speeds.

In order to reuse the battery holder on the bottom plate a battery with the same AA form factor but higher voltage is what we are looking for.
A helpful overview of different batteris with their exact naming can be found in the 
[List of battery sizes](https://en.wikipedia.org/wiki/List_of_battery_sizes) on Wikipedia. Here we can see that the 14500 recharchable 
[Lithium-ion battery](https://en.wikipedia.org/wiki/List_of_battery_sizes#Lithium-ion_batteries_(rechargeable)) provides 3.7 V with a capacity of 
approximately 900 mAh. This means that four batteries of these provide 14.8 V which is right below the maximum 15 V that the motor driver can handle.
Although the capacity is lower compared to non recharchable AA batteries (2700 mAh), these type of batteries can be reused multiple times when charged
with a suitable battery charchger such as the [Nitecore UMS4](https://charger.nitecore.com/product/ums4) or the 
[XTAR VC4](https://www.xtar.cc/product/XTAR-VC4-Charger-20.html). DiffBot uses four 14500 batteries from [Trustfire](https://www.trustfire.com/products/trustfire-14500-900mah-battery) with 3.7 V and 900 mAh.
