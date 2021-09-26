[toc]



# Getting started

This note is written mainly for learning Feedback Control via reading *Feedback Control of Dynamic Systems* and some papers. The pictures, if not marked specially, are from the original book.

Thanks for fellow xzy's support.

​																																													*Written by Bingchuan Wei*



# Week1

## Basic Ideas for Feedback Control
1. Control is the process of making a system variable adhere to a particular value, called the **reference value**. A system designed to follow a changing reference is called **tracking control or a servo**. A system designed to maintain an output fixed regardless of the disturbances present is called a **regulating control or a regulator**.
2. A simple feedback system consists of the **process **whose output is to be controlled, the **actuator/plant** whose output causes the process output to change, a reference command signal, and output sensors that measure these signals, and the **controller** that implements the logic by which the control signal that commands the actuator is calculated. 
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

$$
\frac{\partial^4 w}{\partial x^4} +\rho\frac{\partial^2 w}{\partial t^2} = 0
$$

To simplify the distributed parameter, we can regrad the system as two or more rigid bodies connected by springs, the result is sometimes called a **lumped parameter model**.

<img src="FeedbackControl.assets/Screen Shot 2021-04-09 at 6.44.32 PM.png" alt="Screen Shot 2021-04-09 at 6.44.32 PM" style="zoom:25%;" />

### Operational amplifier

Operational amplifier is basic in circuit system.A operation amplifier's symbol and its schematic are shown below:

<img src="FeedbackControl.assets/Screen Shot 2021-04-09 at 7.01.38 PM.png" alt="Screen Shot 2021-04-09 at 7.01.38 PM" style="zoom:25%;" />

The third picture means the v+ is connected to the GND.

Basic equation of operational amplifier are:
$$
i_{+} = i_{-} = 0 \\
v_{+} -v_{-} = 0
$$





## Dynamic Reponse

### Exponential signal response

The dynamic response for a singal with the form $u(t)=e^(s_0t)$ is very important, and is closely connected with the frequency response.
$$
y(t) = \int_{-\infty}^{+\infty}h(\tau)u(t-\tau)d\tau = \int_{-\infty}^{+\infty}h(\tau)e^{-s_0\tau}d\tau e^{s_0t}=H(s_0)e^{s_0t}
$$
The frequency reponse 
$$
u(t) = A\cos(\omega t)\\
y(t) = A\frac{H(j\omega)e^{j\omega t}+H(-j\omega)e^{-j\omega t}}{2}=AM\cos(\omega t+\phi)
$$
Where $M =|H(j\omega)|, \phi=\ang H(j\omega)$

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

<img src="FeedbackControl.assets/Screen Shot 2021-04-12 at 7.23.01 AM.png" alt="Screen Shot 2021-04-12 at 7.23.01 AM" style="zoom: 50%;" />

$G_1$is called **Forward Gain**, $G_1G_2$is called **Loop Gain**. Set the s = 0, we get **DC gain**, which is the gain for a step input.

The diagram can be transformed to facilitate the analysis

<img src="FeedbackControl.assets/Screen Shot 2021-04-12 at 7.31.55 AM.png" alt="Screen Shot 2021-04-12 at 7.31.55 AM" style="zoom:33%;" />

The feedback in (c) is called **Unity Feedback**

### Second order system and time domain specifications

The standard form of the second order system is
$$
H(s) = \frac{\omega_n^2}{s^2+2\zeta\omega_ns+\omega_n^2}
$$
where $\zeta$ is the damp ratio, $\omega_n$ is the natural frequency.  $\zeta < 1$ is what we call **Lightly Damped** system

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

   Basically, a zero in $s_0$ blocks the input of generalized frequency $s_0$ , while the pole decieds the natrural of unfored behavior of the system which is called **Mode**.(We call impulse response as **natural response**) 

   At the level of transient analysis, the zeros exert their influence by modifying the coefficients of the **Mode**. In general, a zero near a pole reduces the amount of that term in the total response.

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

   For some explanation, see https://courses.engr.illinois.edu/ece486/fa2017/documents/lecture_notes/effects_zero_pole.pdf

   

   

   

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

<img src="FeedbackControl.assets/Screen Shot 2021-04-12 at 3.30.18 PM.png" alt="Screen Shot 2021-04-12 at 3.30.18 PM" style="zoom:50%;" />

If $D_{ol} = \frac{c(s)}{d(s)},G(s) = \frac{b(s)}{a(s)}$, the open-loop gain is $D_{ol}G = \frac{bc}{ad}$

What matters is that the poles in d(s) **can not** be cancelled out by the zero of b(s), even if it is reasonable in mathematics, a slight noise will cause the system break down. We can conclude that

> An open-loop structure *can not* be used to make an unstable plant to be stable, and therefore cannot be used if the plant is already unstable.

Another fatal point of open-loop is that the controller has nothing to do with the disturbance W(s), so the loop-control can't be used as a regulator .



The general structure of feedback system is :

<img src="FeedbackControl.assets/Screen Shot 2021-04-12 at 3.12.50 PM.png" alt="Screen Shot 2021-04-12 at 3.12.50 PM" style="zoom:50%;" />

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

In the general tracking problem, the reference input can often be adequately approximated as if it were a polynomial in time. So it's useful to analyze **steady-state errors in stable systems with polynomial inputs**.

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

4. The break-in point and breakaway points satisfy the equation $\frac{dK}{ds} = 0$, and K must be positive.
5. Departure angle and arrival angle.
6. The intersection point with imaginary axis: Routh Criterion.

These guildlines can only limit the root locus to some types, but not give locus definitely. 

For now, Matlab will draw root locus perfectly so the above rules seem to be useless, but these rules give us a new perspective to check our controller. For example, when we add a zero near the pole, according to the rule of departure angle we may well expect that root will be shifted to LHP.

### Design using dynamic compensation

Using root locus method, we can choose a propotional gain to move poles. However, for many processes, it's not enough changing gain. Under these situations, it's nessasary to change the prossess itself, which is called **compensation**. Three main compensations are used:

- Lead compensation: approximates the function of PD control and acts mainly to speed up a response by lowering rise time and decreasing the transient overshoot.
- Lag compensation: approximates the function of PI control, and is usually used to improve the steady-state accuracy of the system. 
- Notch compensation: used to achieve stability for systems with lightly damped flexible modes.

$$
D_c(s) = \frac{s+z}{s+p}
$$

is called lead compensation and lag compensation for z < p and z >p respectively. They get their name by the phase impact on frequency analysis. The compensation introduces a phase change by
$$
\phi = \tan^{-1}(\frac{\omega}{z})-\tan^{-1}(\frac{\omega}{p})
$$


#### Lead compensation

For PD control, noise from the sensor may be amplified, so lead compensation is proposed. If p is not that far from the z, then the total effect of lead compensation is not a pure derivative. So there is a tradeoff for p. 

#### Lag Compensation

Though lag compensation has the same form with lead compensation, they differ with each other by the choose of z and p. Lead compensation use such z and p as to cancell the pole of the plant and get a higher damping ratio. Lag compensation is usually used after the lead compensation to lower the steady state error by elevate the DC gain. By choosing z and p far smaller than natural frequency, lag compensation don not destroy our effect in lead compensation What we want is just the ratio of z/p.

#### Notch compensation

Notch compensation is used to prevent the resonance in system by changing their phase.

### Successive Loop Closure

Sometimes it's useful to introduce more than one feedback loop, by introducing inner and outer loop, we can design more stable system. For example:

<img src="FeedbackControl.assets/Screen Shot 2021-09-17 at 9.29.21 AM.png" alt="Screen Shot 2021-09-17 at 9.29.21 AM" style="zoom:33%;" />

## The Frequency-Response Design Method

The frequency-response method is the most-widely used method in feedback control system for the effect of alleviating the uncertainties. It is also cost-effective since the method only needs the response of sinusoidal input without the knowledge of the poles and zeros for the system.

### Frequency Response

A linear system’s response to sinusoidal inputs is called the system’s **frequency response**.

The first thing we see is the frequency response of the linear constant system with transfer function G(s):
$$
Input: u(t) = A\sin(\omega_0t)1(t)\\
U(s) = \frac{A\omega_0}{s^2+\omega_0^2}\\
Y(s)=G(s)U(s)=\sum_{i=1}^{n}\frac{\alpha_i}{s-p_i}+\frac{\alpha_0}{s+j\omega_0}+\frac{\alpha_0^*}{s-j\omega_0},where \quad |\alpha_0|=\frac{A}{2}|G(j\omega_0)|\\
y(t) = \sum_{i=1}^{n}e^{p_it}+2|\alpha_0|\cos(\omega_0t+\phi)=AM\cos(\omega_0t+\phi)\\
M=|G(j\omega_0)|\\
\phi= \angle G(j\omega_0)
$$


The important part is the frequency response of such a system will converge to a sin term if the system is stable. M and $\phi$ are determined totally by the input frequency. By measuring the response of different input frequency, we can get the features of system.

Two characteristic value for frequency-response is **bandwidth** and **resonant peak**. Bandwidth can be defined as the frequency where magnitude drops to a certain ratio. It can be used to measure the response speed, The resonant peak can be used to measure the overshoot.

### Bode Graph

The idea in Bode’s method is to plot magnitude curves using a logarithmic scale and phase curves using a linear scale, based on
$$
\lg(Me^{i\phi})=\lg(M)+i\phi\lg(e)
$$
If we use db as the unit for magnitude, the graph becomes the magnitude in decibels versus $\lg(\omega)$,  since we are not actually drawing the power in the definition of the db, so the unit for the |G| axis is usually 20db.

The unit of $j\omega$ axis can be **decade**(10,100,1000...), or **octave**(2,4,8...).

The draw Bode Graph, a good way is to write the transfer function in the bode form:
$$
KG(s) = K_0\frac{(j\omega\tau_1 + 1)(j\omega\tau_2 + 1)\cdots}{(j\omega)^n(j\omega\tau_a + 1)(j\omega\tau_b + 1)\cdots}\\
\lg(KG(s)) = \lg K_0 +\lg (j\omega\tau_1 + 1) + \lg (j\omega\tau_1 + 1)-\lg (j\omega\tau_a + 1)- \lg (j\omega)^n\cdots
$$
We can analyze the result by dividing the result into three classes, for each class, what matters is **Asymptote**, **Slope on the trasition point**, **Value on the trasition point** :

1. $K_0(j\omega)^n$

   $\lg K_0(j\omega)^n = \lg K_0 + n\log (j\omega)$

   The graph for this case is a line with slope of n or 20*n(db/decade)

2. $(j\omega\tau_1 + 1)$

   $(j\omega\tau_1 + 1) \approx 1 ,\omega\approx0,(j\omega\tau_1 + 1)\approx j\omega\tau_1,\omega  \to \infty$

   The breakpoint $\omega = 1 /\tau_1$ is a trasition point with the slope of 1 or 20(db/decade)

   <img src="FeedbackControl.assets/Screen Shot 2021-04-23 at 8.46.47 AM.png" alt="Screen Shot 2021-04-23 at 8.46.47 AM" style="zoom:33%;" /><img src="../../../../../../Library/Application Support/typora-user-images/Screen Shot 2021-04-23 at 8.47.14 AM.png" alt="Screen Shot 2021-04-23 at 8.47.14 AM" style="zoom:33%;" />

3. $((\frac{j\omega}{\omega_n})^2+2\zeta(\frac{j\omega}{\omega_n}+1)$

   The breakpoint $\omega = \omega_n$ with the slope of 2 or 40 (db/decade)

Since the phase angle are sumed for the composite curve, we can draw approximate graph for each interval. 

All above discussion are based on a fact that $\tau > 0$, which is not always the case. For system with $\tau_i > 0$, namely, all zeros and poles in LHP, the phase in $0^{\circ}$ when $\omega = 0$. Such systems are called **Minimum-Phase Systems**. For system wich zeros or poles in RHP, the initial phase with not be zero ,which is called **Non-Minimum-Phase Systems**

### Steady-State Error

When $\omega \approx 0$, we have 
$$
KG(\omega) \approx K_0(j\omega)^n
$$
Compare the form with 
$$
K_n = s^nD_{cl}G(s)
$$
We can find that K at n =0,-1,-2 equals $K_p,K_v,$etc.

For a simple system like:

<img src="FeedbackControl.assets/Screen Shot 2021-04-23 at 10.35.06 AM.png" alt="Screen Shot 2021-04-23 at 10.35.06 AM" style="zoom:33%;" />

 From the Root locus, we know that the neurtral stability condition is 
$$
|KG(s)| = 1 \quad \angle G(s) = 180^{\circ}
$$
According to Bode Graph, we can find the $\omega$ satisfies the angle condition, and we can draw a series lines with different K to find the neutral stability K. However, the grapha is not rigorous, for which we broach Nyquist criterion.

### The Nyquist Stability Criterion

The Nyquist Criterion gives a perfect clarification of the relationship between the open-loop frequency response and  the number of closed-loop poles of the system in the RHP. It is based on the **argument principle**.

> **argument principle**:
>
> A contour map of a complex function will encircle the origin (Z-P) times, where Z is the number of zeros and P is the number of poles of the function inside the contour.

The method of drawing a contour and analyze the contour map of the transfer function is called a **contour evaluation**. For a system with $\mathcal{J}(s) = \frac{KG(s)}{1+KG(s)}$, we do a contour evaluation for $KG(s)$, and check the encirclements of -1. We can get the information of RHP zeros nad poles.This method is called **Nyquist Plot** or **Polar Plot**.

More often, engineers want to get the K range where the system is stable. To do this, we can do contour evaluation for G(s) and to check its encirclements of -1/K. Since the poles of G(s) can always be gotten, we calculate the (N+P) to get the zeros of the system, which is the poles of the original system.

### Stability Margins

The **Gain Margin(GM)** is the **factor** by which the gain can be increased (or decreased in certain cases) before instability results based on the current state if the argument condition is satisfied. If |GM| > 0, the system is stable.

The **Phase Margin(PM)** is the amount by which the phase exceeds $180^{\circ}$ if value condition is satisfied. 

The term **crossover frequency**, is often used to refer to the frequency at which the magnitude is unity, or 0 db.

<img src="FeedbackControl.assets/Screen Shot 2021-04-26 at 1.56.02 PM.png" alt="Screen Shot 2021-04-26 at 1.56.02 PM" style="zoom:33%;" />

GM and PM are clear both on Bode plot and Nyquist plot.

 ### GM and PM limitations

For any first- or second-order system. $GM = \infty$ since the phase never crosses the $180^{\circ}$ line. This just means that GM is no longer a useful parameter. For high order system, the phase may cross the line for many times, where PM and GM need to be clarified.

**Vector Margin** is also introduced to remove the ambiguity. 







# Week 4

## State-Space Design

The basic aim for state-space design is to determine the compensation.

*State-Variable Form*
$$
\dot{\bold{x}} = \bold{Ax}+\bold{Bu}\\
\bold{y} = \bold{Cx}+\bold{Du}
$$
A is called system matrix, B is called input matrix, C is called output matrix, D is called direct transmission term.

For a SISO systme, u is a scalar, so is y.

```matlab
sys = ss(A, B, C, D)
step(sys)
```

We can transform between ss description and tf description 

```matlab
[num den] = ss2tf(A,B,C,D)
[z p k] = ss2zp(A,B,C,D)
[z p k] = tf2zp(num, den)
```

### Transfer function to State-variable form

First thing we want to know to is how to transfer from transfer function to a state space. For a transfer function, the state space is not unique, there are several basic forms for choosing state variables, which are called canonical forms.

For a transfer funciton 
$$
G(s) = \frac{b(s)}{a(s)} = \frac{b_1s^{n-1}+b_2s^{n-2}+\cdots+b_n}{s^n+a_1s^{n-1}+\cdots+a_n}
$$
We have https://en.wikibooks.org/wiki/Control_Systems/Standard_Forms

A natural idea is: whether a state-variable form can be transformed to one of the above canonical forms?

#### Controllability

For controller canonical form, we can explictyly give the transformation matrix if the  **controllability matrix**
$$
C = [B\quad AB\quad \cdots \quad A^{n-1}B]
$$
is nonsingular. We call this state-variable form **Controllable**.

Once a state-space is chosen, you cannot change its controllablity using nonsingluar linear transformation.

For modal form, we see it's equvalent to the diagonalizablity of A. 

#### Observability

For observer canonical form, we can transform to it if **observability matrix**
$$
\mathcal{C} = \begin{pmatrix}
C \\
CA \\
\vdots \\
CA^{n-1}
\end{pmatrix}
$$
is nonsingular(or full rank for column )

### State-variable form to transfer function

We apply Laplace transformaition to the state-variable form 
$$
G(s) = C(sI-A)^{-1}B+D
$$
What we are intererted is the state-variable form of zeors and poles.

If $p_i$ is a pole of the system, it means there exists $x_0$ makes 
$$
x(t) = Ke^{p_it}
$$
with u = 0, namely **$p_i$ is the eigenvalue of A**

If $z_i$ is a zero of the system, it means that for input with generalized frequency $z_i$, output $y \equiv 0 $

We finally get 
$$
\begin{pmatrix}
z_iI-A & -B \\
C & D
\end{pmatrix}
\begin{pmatrix}
x_0 \\
u_0
\end{pmatrix} = 0
$$

### Control Law Design for full-state feedback

<img src="FeedbackControl.assets/Screen Shot 2021-09-15 at 6.59.40 AM.png" alt="Screen Shot 2021-09-15 at 6.59.40 AM" style="zoom: 50%;" />

This is the basic structure of a feedback control system.

Assume we've got enough sensors that we know $\vec{x}$ explicitly, which is called **Full State**，and suppose no reference point is given . Then the diagram is simplified to 

<img src="FeedbackControl.assets/Screen Shot 2021-09-15 at 7.31.54 AM.png" alt="Screen Shot 2021-09-15 at 7.31.54 AM" style="zoom:33%;" />

Then we can caculate the poles of system using 
$$
\det(sI - (A-BK)) = 0
$$
since we have n free parameteres in K, we can shift poles to where I want with large freedom. Two ways can be used to determine  K under a given set of poles. One is transform the state-space to control canonical forms, the other is Ackerman method.
$$
K = [0 \cdots 0 \quad1]C^{-1}\alpha_c(A) \\
\alpha_c(A) = A^n+p_1A^{n-1}+\cdots+p_nI \\
C = [B \quad AB \cdots A^{n-1}B]
$$
For a proof, see https://en.wikipedia.org/wiki/Ackermann%27s_formula

#### More on controllablity

When system is not controllable, we can not shift its poles at will. **Uncontrollable systems** have certain modes, or subsystems, that are unaffected by the control. This usually means that parts of the system are physically disconnected from the input(Like there is a zero in B). No mathematical test can replace the control engineer’s understanding of the physical system. Often the physical situation is such that every mode is controllable to some degree, and, while the mathematical tests indicate the system is controllable, certain modes are so weakly controllable that designs to control them are virtually useless. The system has to work harder and harder to achieve control as controllability slips away and thus drive the actuators into saturation. To move the poles a long way requires large gains.

#### Introduce reference point

Recall that a reference point is the value we want y to track. In steady state, we want
$$
\vec{x} =  \vec{x}_{ss} \quad \vec{\dot{x}} = 0\\
y = y_{ss} = r_{ss} \\
u = u_{ss} \\
$$
Suppose that $\vec{x}_{ss} = N_xr_{ss}, u_{ss} = N_ur_{ss}$, plug these in our plant, we get
$$
\begin{pmatrix}
A & B\\
C & D
\end{pmatrix}
\begin{pmatrix}
N_x\\
N_u
\end{pmatrix} =\begin{pmatrix}
0\\
1
\end{pmatrix} 
$$
It's natural for us to set $u = u_{ss} - K(\vec{x}-\vec{x}_{ss})$
$$
u = -K\vec{x}+(N_u+KN_{x})r
$$
That is <img src="FeedbackControl.assets/Screen Shot 2021-09-15 at 9.15.59 AM.png" alt="Screen Shot 2021-09-15 at 9.15.59 AM" style="zoom:33%;" /> ($\bar{N} = N_u +KN_x$)



or <img src="FeedbackControl.assets/Screen Shot 2021-09-15 at 9.24.50 AM.png" alt="Screen Shot 2021-09-15 at 9.24.50 AM" style="zoom:33%;" />,which is more rubust sometimes.

By now, we have established our control law, if we caculate the zeros of the system
$$
\vec{\dot{x}} = (A-BK)\vec{x}+\bar{N}u
$$
we find its exactly the same as the initial system, that is :

==When full-state feedback is used , the zeros remain unchanged by the feedback.==

#### Choose poles

One problem wasn't solved is where we should place our poles. There are two ways to decide poles, one is to choose a pair of dominant poles to mimic the second-order system, the other is based on LQR.

##### LQR control

For a typical system, we define the tracking error z and loss $\mathcal{J}$
$$
z = C_1x \\
\mathcal{J} = \int_0^{\infty}[\rho z^2+u^2]dt
$$
To minimize loss, we can prove poles are given by *SRL equation*
$$
1+\rho G_0(s)G_0(-s) = 0
$$
where $G_0 = \frac{Z(s)}{U(s)}$

### Estimator design

Estimator is used when not all variables in state space is used for feedback. What we want is to reconstruct the system using part of the observed variable. Namely, **get real $\vec{x}$  from some computed variables**

#### Full-order estimators

If we construct a model with the same dynamics with process, than we can well expect the output $\hat{y}$ from our $\hat{x}$ to be same with that from x, the question is we can't get the inital value of x. Our strategy is to measure output y and make a feedback system to make out estimation to be the same as y, this is called full-order estimators:

<img src="FeedbackControl.assets/Screen Shot 2021-09-17 at 9.48.05 PM.png" alt="Screen Shot 2021-09-17 at 9.48.05 PM" style="zoom:33%;" />

We define error $\bar{x} = x - \hat{x}$,  we can get 
$$
\dot{\hat{x}} = A\hat{x}+Bu+L(y-C\hat{x})\\
\dot{\bar{x}} = (A-LC)\bar{x}
$$
we can choose L in the same way for K and make error to converge. We also have the ackerman formula for s

 We can connect the full state feedback with this to check whether our estimate works.

<img src="FeedbackControl.assets/Screen Shot 2021-09-17 at 10.01.24 PM.png" alt="Screen Shot 2021-09-17 at 10.01.24 PM" style="zoom:50%;" />

#### More on observability

Roughly speaking, observability refers to our ability to deduce information about all the modes of the system by monitoring only the sensed outputs. Unobservability results when some mode or subsystem is disconnected physically from the output and therefore no longer appears in the measurements.

We have Ackerman formula for observablity
$$
L = \alpha_e(A)\mathcal{C}^{-1}\begin{pmatrix}
0 \\
0 \\
\vdots \\
1
\end{pmatrix}
$$

#### Reduced-Order Estimators

If we've got some good sensors to measure some of the state variables, then not all of them need estimating. The **reduced-order estimator** reduces the order of the estimator by the number (1 in this text) of sensed outputs. The key point in mechanism of reduced-order is to see terms containing measured variables as some kind of input and measurement and use the same techs as    full-order estimator.

#### Estimators with noise

There are always two kinds of noise existing in control: process noise $w$ and sensor noise $\nu$. Taking these into consideration, we get 
$$
\dot{\vec{x}} = A\vec{x}+Bu+B_1w \\
y = C\vec{x} + \nu \\
\dot{\bar{x}} = (A-LC)\bar{x}+B_1w-L\nu
$$
Just like the tradeoff between control effort u and reponse speed r, there is also tradeoff between two kinds of noise. If L is very small, then the effect of sensor noise is removed, but the estimator’s dynamic response will be “slow,” so the error will not reject effects of very well.On the other hand, if is large, then the estimator response will be fast and the disturbance or process noise will be rejected, but the sensor noise, multiplied by L, results in large errors.

Just like what we did in LQR method, we introduce q as the ratio of $w/ \nu$, then choose poles use SRL equation
$$
1+qG_e(-s)G_e(s) = 0
$$
where $G_e$ is  the transfer function from the process noise to the sensor output.

