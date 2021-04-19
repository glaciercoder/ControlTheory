# Getting started
This note is written mainly for learning Feedback Control via reading *Feedback Control of Dynamic Systems* and some papers. The pictures, if not marked specially, are from the original book.

Thanks for fellow xzy's support.



# Week1

## Basic Ideas for Feedback Control
1. Control is the process of making a system variable adhere to a particular value, called the **reference value**. A system designed to follow a changing reference is called **tracking control or a servo**. A system designed to maintain an output fixed regardless of the disturbances present is called a **regulating control or a regulator**.
2. A simple feedback system consists of the **process (or plant)**whose output is to be controlled, the **actuator** whose output causes the process output to change, a reference command signal, and output sensors that measure these signals, and the **controller** that implements the logic by which the control signal that commands the actuator is calculated. 
3. A well-designed feedback control system will be **stable, track a desired input or setpoint , reject disturbances, and be insensitive (or robust) to changes in the math model used for design**.
4. The theory and design techniques of control have come to be divided into two categories: **classical control** methods use Laplace transforms (or z-transform) and were the dominant methods for control design until **modern control** methods based on ODEs in state form were introduced into the field starting in the 1960s. 



## Dynamic models
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
The diagonal rotors(eg. 1 and 3) are driven in the same direction(CW or CCW), the reaction the rotor exerts  on the drone consists of the lift force and the reaction torque. To control Pitch and roll, the diagonal rotor must speed up and slow down simultaneously. To control yaw, the diagonal rotor must all speed up or slow down.

The MUX(multiplexer) block in simulink can combine the signal from multi channl and draw them in a same diagram.

### Distributed Parameter Systems
For systems with flexible parts in it(like a rod), the dynamic equation will always a high order equation. eg.

<img src="/Users/glacier/Desktop/SRT/Agile_Vehicle/my/control_theory/ControlTheory/FeedbackControl.assets/Screen Shot 2021-04-09 at 6.36.39 PM.png" alt="Screen Shot 2021-04-09 at 6.36.39 PM" style="zoom:25%;" />



Where 
$$
\frac{\partial^4 w}{\partial x^4} +\rho\frac{\partial^2 w}{\partial t^2} = 0
$$

To simplify the distributed parameter, we can regrad the system as two or more rigid bodies connected by springs, the result is sometimes called a **lumped parameter model**.

<img src="/Users/glacier/Desktop/SRT/Agile_Vehicle/my/control_theory/ControlTheory/FeedbackControl.assets/Screen Shot 2021-04-09 at 6.44.32 PM.png" alt="Screen Shot 2021-04-09 at 6.44.32 PM" style="zoom:25%;" />

### Operational amplifier

Operational amplifier is basic in circuit system.A operation amplifier's symbol and its schematic are shown below:

<img src="/Users/glacier/Desktop/SRT/Agile_Vehicle/my/control_theory/ControlTheory/FeedbackControl.assets/Screen Shot 2021-04-09 at 7.01.38 PM.png" alt="Screen Shot 2021-04-09 at 7.01.38 PM" style="zoom:25%;" />

The third picture means the v+ is connected to the GND.

Basic equation of operational amplifier are:
$$
i_{+} = i_{-} = 0 \\
v_{+} - v_{-} = 0
$$





## Dynamic Reponse

###   Bode Plots and Matlab implementation 

The plot where the frequency response of $ \cos(\omega t)$ is drawn with amplitude and phase apart is called bode plot. The following code is an example for bode plot in Matlab

```Matlab
k = 1;
tf=(‘s’);
sysH = 1/(s+k);
function
w = logspace(-2,2);
[mag,phase] = bode(sysH,w);
loglog(w,squeeze(mag));
semilogx(w,squeeze(phase));
% define Laplace variable
% define system by its transfer
% set frequency w to 50 values from 10^-2 to 10^2
% compute frequency response
% log–log plot of magnitude
% semi-log plot of phase
```

The usage of several functions :

[logspace](https://www.mathworks.com/help/matlab/ref/logspace.html?searchHighlight=logspace&s_tid=srchtitle)

[bode](https://www.mathworks.com/help/control/ref/lti.bode.html?s_tid=srchtitle)

[squeeze](https://www.mathworks.com/help/matlab/ref/squeeze.html?s_tid=srchtitle)

### Some basic functions of Laplace Transform in Matlab

```matlab
%code block 1
num = 2;                    % form numerator
den = poly([-2;-1;-4]);     % form denominator polynomial from its roots
[r,p,k] = residue(num,den); % compute the residues

%code block 2
s=tf(‘s’); % define Laplace variable 
sysH=0.001/(s^2+0.05*s); % form transfer function
p=pole(sysH);     % compute poles
[z,k]=zero(sysH); % compute zeros and transfer function gain

%block 3
s=tf('s'); % define Laplace variable
sysG=0.0002/s ^2; % define system by its transfer function 
t=0:0.01:10; % set up time vector with dt = 0.01 sec
% pulse of 25N, at 5 sec, for 0.1 sec duration
u1=[zeros(1,500)
 25*ones(1,10)
zeros(1,491)];    % pulse input
[y1]=lsim(sysG,u1,t); % linear simulation
ff=180/pi; % conversion factor from radians to degrees
y1=ff*y1; % output in degrees
plot(t,u1);    % plot input signal
plot(t,y1);   % plot output response
```

The usage of several functions :

[poly](https://www.mathworks.com/help/matlab/ref/poly.html)     [roots](https://www.mathworks.com/help/matlab/ref/roots.html?searchHighlight=roots&s_tid=srchtitle)

[residue](https://www.mathworks.com/help/matlab/ref/residue.html)

[zero](https://www.mathworks.com/help/control/ref/lti.zero.html?searchHighlight=zero&s_tid=srchtitle)      [pole](https://www.mathworks.com/help/control/ref/lti.pole.html?searchHighlight=pole&s_tid=srchtitle)

[lsim](https://www.mathworks.com/help/control/ref/lti.lsim.html?searchHighlight=lsim&s_tid=srchtitle)

### System Modeling Diagrams

<img src="/Users/glacier/Desktop/SRT/Agile_Vehicle/my/control_theory/ControlTheory/FeedbackControl.assets/Screen Shot 2021-04-12 at 7.23.01 AM.png" alt="Screen Shot 2021-04-12 at 7.23.01 AM" style="zoom: 50%;" />

$G_1$is called **Forward Gain**, $G_1G_2$is called **Loop Gain**. Set the s = 0, we get **DC gain**, which is the gain for a step input.

The diagram can be transformed to facilitate the analysis

<img src="/Users/glacier/Desktop/SRT/Agile_Vehicle/my/control_theory/ControlTheory/FeedbackControl.assets/Screen Shot 2021-04-12 at 7.31.55 AM.png" alt="Screen Shot 2021-04-12 at 7.31.55 AM" style="zoom:33%;" />

The feedback in (c) is called **Unity Feedback**

### Second order system and time domain specifications

The standard form of the second order system is
$$
H(s) = \frac{1}{s^2+2\zeta\omega_ns+\omega_n^2}
$$
where $\zeta$ is the damp ratio, $\omega_n$ is the natural frequency.

The standard impulse response :
$$
h(t) = \frac{\omega_n}{\sqrt{1-\zeta^2}}e^{-\sigma t}\sin(\omega_d t)
$$
The standard step response:
$$
y(t) = 1-\frac{e^{-\sigma t}}{\sqrt{1-\zeta^2}}\cos(\omega_dt-\beta)
$$
where 
$$
\sigma = \zeta\omega_n\\
\omega_d=\sqrt{1-\zeta^2}\omega_n\\
\beta = \arctan(\frac{\zeta}{\sqrt{1-\zeta^2}})
$$
According to the standard response, we can analyze the domain specifications

1. Rise Time

   For a second-order system with no zeros, we have
   $$
   t_r = \frac{1.8}{\omega_n}
   $$
   
2. Overshoot and Peak time

   Analytically, we have
   $$
   t_p = \frac{\pi}{\omega_d}\\
   M_p = e^{-\pi\tan\beta}
   $$
   
3. Settling Time

   Define the settling time as that value of when the decaying exponential reaches 1%:
   $$
   t_s = \frac{4.6}{\sigma}
   $$
   

   In s-plane, the above relationship can help us to choose the reasonable poles location.

   ### Effects of Zeros and Additional Poles

   At the level of transient analysis, the zeros exert their influence by modifying the coefficients of the exponential terms whose shape is decided by the poles, which is called **Mode**. In general, a zero near a pole reduces the amount of that term in the total response.

   The analysis of zeros is deviding the tf into the two parts, the former being the initial response, the latter regarded as the derivative of the inital response multiplied by a constant.

   The zero in the right half plane is called **RHP** or **nonminimum-phase zero**

   Placing the complex zeros near the locations of the lightly damped poles may provide sufficient improvement in step response performance. 

   We can get the following experience:

   1. A zero in the left half-plane (LHP) will increase the overshoot if the zero is within a

      factor of 4 of the real part of the complex poles. 								.

   2. A zero in the RHP will depress the overshoot (and may cause the step response to

      start out in the wrong direction).

   3. An additional pole in the LHP will increase the rise time significantly if the extra pole is

      within a factor of 4 of the real part of the complex poles.

   

   

   

   

# Week 2

   ## Stability

   Thic section mainly foucs on the LTI system's stability.

   A basic condition for the stability is:

   >An LTI system is said to be stable if all the roots of the transfer function denominator polynomial have negative real parts , and is unstable otherwise.

   ### BIBO Stability

   > A system is said to have **bounded input–bounded output (BIBO) stability** if every bounded input results in a bounded output.

   We can get a equivalent condition  of BIBO:

   > The system with impulse response being h(t) is BIBO-stable if and only-if:
   > $$
   > \int_{-\infty}^{\infty}|h(\tau)|d\tau < \infty
   > $$
   > 

   If all poles are LHP, the stability is called **internal stability**. If there are no repeated pure imaginary pole, the stability is called **neutrally stable**.

   

   The most simple necessary condition 

   > A necessary (but not sufficient) condition for stability is that all the coefficients of the characteristic polynomial be positive.

   Attention must be paid in using this criterion is than all coefficients must be positive. If 0 and negative appears, another methoed must be taken.

   The sufficient and necessary condition of BIBO stable is given by Routh-Hurwitz criterion. Routh-Hurwitz criterion not only gives the result of stability, but gives the number of LHP roots.

   [Routh-Hurwitz criterion](https://en.wikipedia.org/wiki/Routh%E2%80%93Hurwitz_stability_criterion)

   

   Matlab can be used to draw the implicit function using [fimplicit](https://www.mathworks.com/help/matlab/ref/fimplicit.html)

   

   ## A First Analysis of Feedback 

This part mainly discusses LTI SISO system.

The general structure of open-loop system is :

<img src="/Users/glacier/Desktop/SRT/Agile_Vehicle/my/control_theory/ControlTheory/FeedbackControl.assets/Screen Shot 2021-04-12 at 3.30.18 PM.png" alt="Screen Shot 2021-04-12 at 3.30.18 PM" style="zoom:50%;" />

If $D_{ol} = \frac{c(s)}{d(s)},G(s) = \frac{b(s)}{a(s)}$, the open-loop gain is $D_{ol}G = \frac{bc}{ad}$

What matters is that the poles in d(s) **can not** be cancelled out by the zero of b(s), even if it is reasonable in mathematics, a slight noise will cause the system break down. We can conclude that

> An open-loop structure *can not* be used to make an unstable plant to be stable, and therefore cannot be used if the plant is already unstable.

Another fatal point of open-loop is that the controller has nothing to do with the disturbance W(s), so the loop-control can't be used as a regulator .



The general structure of feedback system is :

<img src="/Users/glacier/Desktop/SRT/Agile_Vehicle/my/control_theory/ControlTheory/FeedbackControl.assets/Screen Shot 2021-04-12 at 3.12.50 PM.png" alt="Screen Shot 2021-04-12 at 3.12.50 PM" style="zoom:50%;" />

The basic equations are 
$$
Y_{cl} = \frac{D_{cl}G}{1+D_{cl}G}R+\frac{G}{1+D_{cl}G}W-\frac{D_{cl}G}{1+D_{cl}G}V\\
E_{cl} = \frac{1}{1+D_{cl}G}R-\frac{G}{1+D_{cl}G}W+\frac{D_{cl}G}{1+D_{cl}G}V
$$
We define
$$
S = \frac{1}{1+D_{cl}G}\\
\mathcal{J} =\frac{D_{cl}G}{1+D_{cl}G}\\
$$
then
$$
Y_{cl} = \mathcal{J}R+GSW-\mathcal{J}V\\
E_{cl} = SR-GSW+\mathcal{J}V
$$


### The trade-off in the two disturbances 

According to the basic equation, if we increase the $D_{cl}$ to resist the W(s), the disturbance from V(s) will increase.

The method of the dilemma is to design the controller that has different gains in the different frequency. Since the W(s) and V(s) have different properties. The environment disturbances are always of low frequency, somtimes are just a bias, while the sensor disturbances are always of high frequency.

### Sensitivity

We define the sensitivity in open loop system:
$$
S_{G}^{T} = \frac{\frac{dT_{ol}}{T_{ol}}}{\frac{dG}{G}}
$$
Since $T_{ol} = D_{ol}G$, if we take $D_{ol}$ approximately as a constant, we get $S_{G}^{T} = 1$ for open loop system.

Similarly, we can define the same concept in closed-loop system:
$$
S_{G}^{T} = \frac{\frac{dT_{cl}}{T_{cl}}}{\frac{dG}{G}}
$$
We get
$$
S_{G}^{T} = \frac{1}{1+D_{cl}G}
$$
We see $S_{G}^{T}$ is exactly we defined as S in the basic equations of feedback system. we call S as **Sensitivity Function**, and $\mathcal{J}$ as **complementary sensitivity function**.

For the case where there is a non-unity pre filter F (s) following the reference input, R(s) , and non-unity sensor dynamics H (s) , the
equations for the system output and the various sensitivity functions need to be re-derived.

### System Type

In the general tracking problem, the reference input can often be adequately approximated as if it were a polynomial in time. So it's useful to analyze 

**steady-state errors in stable systems with polynomial inputs**.

We have
$$
\mathcal{L}(\frac{t^k}{k!}) = \frac{1}{s^{k+1}}
$$
If there are n integral in $GD_{cl}$, we can set
$$
K_{n} = s^nGD_{cl}
$$
Then the steady-state error for input $\frac{1}{s^{k+1}}$ is
$$
\lim\limits_{t \to +\infty} E(t) = \lim\limits_{s \to 0}\frac{s^n}{s^n+K_n}\frac{1}{s^k}
$$

The system's steady-state error is decided by n and k

| n and k relationship | Steady-state error |
| :------------------: | :----------------: |
|        n > k         |         0          |
|        n < k         |      $\infty$      |
|      n = k = 0       | $\frac{1}{1+K_0}$  |
|    $n = k \neq 0$    |  $\frac{1}{K_n}$   |

 n is called **Systme Type**,     $K_n = \lim\limits_{s \to 0}s^nGD_{cl}$  are called **error constants**

We see that a system has no steady-state error only if system type is higher than the order of reference point.

The other perspective regarding the system type is that for an unit-feedback system, if parameters in Plant change without the moving of zero poles, then the steady-state error will still be zero, which is called **robust property of unity feedback system**

For non-unitfeedback system, system type can be defined via close-loop transfer function $\mathcal{J}(s)$.

<img src="FeedbackControl.assets/Screen Shot 2021-04-14 at 10.14.48 AM.png" alt="Screen Shot 2021-04-14 at 10.14.48 AM" style="zoom: 33%;" />
$$
E(s) = (1-\mathcal{J(s)})R(s)\\
\lim\limits_{t \to +\infty} E(t) = \lim\limits_{s \to 0} \frac{1}{s^k}(1-\mathcal{J(s)})
$$
The critical *s* which makes the steady-state error a constant is defined as the system type.

### System Type for Regulation and Disturbance Rejection

If we set reference to 0, namely, a regulator, then the error is caused by the disturbance W(s). We can take the W(s) as the reference point and define system type the same way above.

### PID Controller

1. P controller

   >$u(t) = k_pe(t)$,$k_p$ for **proportional gain**

   For a simple 2-order system, the system is 0 type. With the increase of $k_p$, the steady-state error will decrease, but the settling time will not be improved. The damping will decrease, which means that the oscillation will increase. System type of disturbance W(s) is also 0, which means that a step disturbance will also cause a steady-state error.

   For systems beyond second order, the situation is more complicated. A higher gain will increase the speed of response but typically at the cost of a larger transient overshoot and less overall damping.

2. I controller 

   > $u(t) = k_I\int_{t_0}^{t}e(\tau)d\tau$, $k_I$ for **integral gain**

   Integral feedback results in zero steady- state output error in both tracking and disturbance rejection. Since addition of integral control to the G(s) typically makes the system a Type 1, the  **robust property ** is reserved.

    However, integral control typically decreases the damping or stability of a system.

3. D controller

   > $u(t) = k_d\dot{e}(t)$, $k_d$ for **derivative gain**

   Derivative control is almost never used by itself; it is usually augmented by proportional control. Since the D controller takes control action based on the trend in the error signal, it is said to have an “anticipatory” behavior.

   Sometimes the D controller is used as below to cancell the abrupt change in W(s):

   <img src="FeedbackControl.assets/Screen Shot 2021-04-14 at 11.17.15 AM.png" alt="Screen Shot 2021-04-14 at 11.17.15 AM" style="zoom:33%;" />



### Feedforward Control

The basic idea of feedforward control is that if we can get the inverse of the DC gain of G(s), we can directly send the error into the plant to compensate for the error as shown below:

<img src="FeedbackControl.assets/Screen Shot 2021-04-14 at 11.43.00 AM.png" alt="Screen Shot 2021-04-14 at 11.43.00 AM" style="zoom:33%;" />

<img src="FeedbackControl.assets/Screen Shot 2021-04-14 at 11.43.17 AM.png" alt="Screen Shot 2021-04-14 at 11.43.17 AM" style="zoom:33%;" />





# Week 3

## The Root-Locus Design Method

For a closed-loop system, the characteristic equation 
$$
1+D_c(s)G(s)H(s) = 0
$$
can be written as **root locus form**:
$$
1+KL(s) = 0
$$
where K is our concerned parameter, 

The **breakaway points**(root move away from the real axis) and the **break-in points**(root move into the real axis) are important in root locus.

Grenerally, it's hard to get the root locus explicitly. Evans proposed a set of rules to draw root locus.

### Guidelines for Determining a Root Locus

If the parameter K is real and positive, we call the locus **positive locus** or **$180^{\circ}$ locus** since the phase of L(s) must be $180^{\circ}$ to satisfy the equation or else we get **negative locus** or **$0^{\circ}$ locus**. We discuss positive locus here.

Set  $L(s) = \frac{b(s)}{a(s)}$, b(s) is a polynomial of m degrees, a(s) is a polynomial of n degrees.

1. The n branches of the locus start at the poles of L(s) and m of these branches end on the zeros of L(s).

2. The loci are on the real axis to the left of an odd number of poles and zeros.

3. The asymptotic line of loci: (n-m) branches of the loci are asymptotic to lines at angles $\phi_l$ radiating out from the point $s = \alpha$ on the real axis, where 

$$
\phi_l = \frac{180^{\circ}+360^{\circ}(l-1)}{n-m},l=1,2,\dots,n-m\\
\alpha = \frac{\sum p_i-\sum z_i}{n-m}
$$

4. 







 















