import control as ct
import matplotlib.pyplot as plt
import numpy as np

#  moment of inertia of the rotor kg.m^2
J = 0.02
# motor viscous friction constant N.m.s
b = 0.1
# Kt = Ke
K = 0.4
# electromotive force constant V/rad/sec
Ke = K
#  motor torque constant N.m/Amp
Kt = K
#  electric resistance Ohm
R = 1
# electric inductance H
L = 0.5
    


def main():
    
    # Electric motor input: voltage, output: speed
    A = [[-b/J,  K/J],
         [-K/L, -R/L]]

    B = [[0],
        [1/L]]

    C = [1, 0]

    D = 0
    
    dt = 0.001
    motor_ss = ct.ss(A,B,C,D)
    print(motor_ss)
    
    T = np.arange(0.0, 3.0, dt)   
    T, yout, xout = ct.forced_response(motor_ss, T=T, U=1, X0=0)
    
    # Motor system response
    plt.plot(T, yout)
    plt.show(block=False)
    
    # Wheel radius m
    radius = 0.0331
    # desired target velocity m/s
    velocity = 0.2
    # angular velocity rad/s
    omega = velocity / radius
    print("w = v/r <-> ", omega, " = ", velocity, " / ", radius)
    
    # PID Controller
    kf = 0.8
    kp = 0.35
    ki = 0.5
    kd = 0.01
    pid_sys = kf * omega +  kp + ki*ct.tf([1], [1, 0]) + kd*ct.tf([1, 0], [1])
    
    
    # Control loop
    motor_sys = ct.tf(motor_ss)
    series_sys = ct.series(pid_sys, motor_sys)
    print(pid_sys, " * ", motor_sys, " = ", series_sys)
    out_sys = ct.feedback(series_sys)
    print(out_sys)
    
    
    
    
    T, yout, xout = ct.forced_response(out_sys, T=T, U=omega, X0=0)
    plt.plot(T, yout)
    print(xout.shape)
    #plt.plot(T, xout[3,:])
    plt.show(block=True)
    
    
    
    
if __name__ == "__main__":
    main()