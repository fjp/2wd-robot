## Robotic Motion and Odometry

This section reviews the basic concepts of distance, time, velocity and acceleration. 
The physics of motion is can be described using calculus, but a computer cannot deal with continuous functions; 
instead, discrete approximations must be used.

Odometry, the fundamental algorithm for computing robotic motion. 
An approximation of the location of a robot can be obtained by repeatedly computing the distance moved and 
the change direction from the velocity of the wheels in a short period of time. 
Unfortunately, odometry is subject to serious errors. 
It is important to understand that errors in direction are much more significant than errors in distance.
