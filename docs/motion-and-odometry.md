## Robotic Motion and Odometry

This section reviews the basic concepts of distance, time, velocity and acceleration. 
The physics of motion is can be described using calculus, but a computer cannot deal with continuous functions; 
instead, discrete approximations must be used.

Odometry, the fundamental algorithm for computing robotic motion. 
An approximation of the location of a robot can be obtained by repeatedly computing the distance moved and 
the change direction from the velocity of the wheels in a short period of time. 
Unfortunately, odometry is subject to serious errors. 
It is important to understand that errors in direction are much more significant than errors in distance.


In the simplest implementation, the speed of the wheels of a robot is assumed to be proportional to 
the power applied by the motors. To improve the accuracy of odometry wheel encoders can be used, 
which measure the actual number of revolutions of the wheels.

### Distance, Velocity and Time

In general, if a robot moves at a constant applied to the motors it causes the wheels to rotate, which in turn causes the robot velocity $v$ for a period of time $t$, the distance $s$ it moves is $s = vt$. When power is to move at some velocity. However, we cannot specify that a certain power causes a certain velocity

- No two electrical or mechanical components are ever precisely identical.
- The environment affects the velocity of a robot because of different friction of the surface
- External forces can affect the velocity of a robot. It needs more power to sustain a specific velocity when moving uphill and less power when moving downhill, because the force of gravity decreases and increases the velocity.
