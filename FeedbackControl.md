# Getting started
This note is written mainly for learning Feedback Control via reading *Feedback Control of Dynamic Systems* and some papers. Thanks for fellow xzy's support.



# Week1

## Basic Ideas for Feedback Control
1. Control is the process of making a system variable adhere to a particular value, called the **reference value**. A system designed to follow a changing reference is called **tracking control or a servo**. A system designed to maintain an output fixed regardless of the disturbances present is called a **regulating control or a regulator**.
2. A simple feedback system consists of the **process (or plant)**whose output is to be controlled, the **actuator** whose output causes the process output to change, a reference command signal, and output sensors that measure these signals, and the **controller** that implements the logic by which the control signal that commands the actuator is calculated. 
3. A well-designed feedback control system will be **stable, track a desired input or setpoint , reject disturbances, and be insensitive (or robust) to changes in the math model used for design**.
4. The theory and design techniques of control have come to be divided into two categories: **classical control** methods use Laplace transforms (or z-transform) and were the dominant methods for control design until **modern control** methods based on ODEs in state form were introduced into the field starting in the 1960s. 

## Dynamic models and simple Matlab implementation
###  Step response with Matlab
```matlab
s=tf(‘s’);  % sets up the mode to define the transfer function
sys = (1/1000)/(s + 50/1000);  % defines the transfer function 
step(500*sys);   % plots the step response for u = 500.
```
[tf() function usage](https://www.mathworks.com/help/control/ref/tf.html)
[step() function usage](https://www.mathworks.com/help/control/ref/lti.step.html?searchHighlight=step&s_tid=srchtitle)

### Collocated control and non-collocated control
The control situation where the sensor and actuator are rigidly attached to one another is called **collocated control**, while the other is called **non-collocated control**
### A brief example of Quadrotor Drone
A simple graph of a quadrotor:

<img src="FeedbackControl.assets/Screenshot_2021-04-09 Feedback Control of Dynamic Systems, 8 e - (8 ed) Gene F Franklin, J David Powell, Abbas Emami-Naeini[...].png" style="zoom: 25%;" />

The sensors used in quadrotor are mainly ultrasound(to get the distance from a surface), camera(to detect the motion of the surrounding to judge the motion of drone), IMU, pressure sensor(to get the altitude).
The diagonal rotors(eg. 1 and 3) are driven in the same direction(CW or CCW), the reaction the rotor exerts  on the drone consists of the lift force and the reaction torque. To control Pitch and roll, j







