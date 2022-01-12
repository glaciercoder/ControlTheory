# Background

While studying general principles and methods for controlling and modelling, there is a need to learn some special methods that is widely used and researched.  Sliding Mode Control(SMC) is one kind of so called robust control.

SMC belongs to **Variable structure control**. A variable structrue control system(VSCS) has a so called **switching function** which chooses the controller based on input. 

Key features of SMC is its **disturbance rejection** and **order reduction**



# Basic ideas

The design for SMC consists of two parts:

1. Design of the sliding surface, which is based on our desired dynamic performance
2. Synthesis of the control law, which make our surface at least locally attractive.

The dynamic process of a VSCS of SMC is divided into two parts: from inital state to sliding surface, called **reaching phase**, on sliding surface, called **sliding motion**, consists **ideal sliding motion** if there is no overshoot or high frequency oscillations and **chattering**. In system which can not accept chattering, we use continous control and confine state to a layer of sliding surface, which is called **pesudo-sliding motion**

## Problem posed

Consider following LTI system with pertubation:
$$
\dot{x} = Ax+Bu+f(t,x,u),A\in \mathbb{R}^{n\times n} , B\in\mathbb{R}^{n\times m},1\leq m < n \\
f : \mathbb{R}\times \mathbb{R}^{n} \times \mathbb{R}^{m} \mapsto \mathbb{R}^{n} \text{is bounded}
$$

We can suppose B with full rank without generality

Let $s:\mathbb{R}^{n} \mapsto \mathbb{R}^{m},s(x) = Sx$ , S is full rank, it defined a hyper plane:
$$
\mathcal{S} = \{x\in \mathbb{R}^{n}:s(x) = 0\}
$$
s is called switching function

>Definition: If  $ \exists \quad t_s < \infty$, the solution of our dyanmic equation satisfies 
>$$
>s(t) = 0,\forall t \geqslant t_s
>$$
>then a ideal sliding motion is said to take place from $t_s$



## Equivalent control

Our dynamic equation can be written as 
$$
\dot{x} = F(t,x)
$$
Since normally we choose discontinuous control law, the dynamic equiation is not ensure with existence and uniqueness. The way to solve this to introduce equivalent control. 

<img src="Sliding_Mode_Control.assets/Screen Shot 2021-10-08 at 9.26.41 AM.png" alt="Screen Shot 2021-10-08 at 9.26.41 AM" style="zoom:33%;" />

we substitue the derivative near the sliding plane with the linear combination of its sided derivative to make it tagent to sliding plane. Ignoring all these stuff, on average, our final result can be get using a control signal:
$$
\dot{x} = Ax+Bu \\
S\dot{x} = SAx+SBu= 0,\forall t\geqslant t_s \\
$$
By choosing S, we make SB nonsigular, then **Equivalent control** is defined:
$$
u_{eq} = -(SB)^{-1}(SA)x
$$
Then ideal  sliding motion equation:
$$
\dot{x} = (I_n-B(SB)^{-1}S)Ax
$$
Define $P_s = I_n-B(SB)^{-1}S$

We have two important relationship
$$
SP_s = 0 \quad P_{s}B = 0
$$
==Attention:== the concept of equivalent control is to justify the existence and uniqueness of steaty sliding motion, and can not be put into practice due to the exitence of unknown perturbation.

Equivalent control is of great significance since real control system has a bandwidth, the actual control signal 
$$
u(t) = u_{eq} + (u(t)- u_{eq}(t))
$$
The last term may be cut off by the bandwidth, so the actual control is just equivalent control. Actually, $u_{eq}$ is also the secret to cancellation of the unknown disturbance.



## Order reduction and Matched uncertainty rejection

Model order reduction(MOR) and insensitivity to uncertainty are two feature making VSCS powerful. Now we can give it a proof for SMC.

Consider a special pertubation in our dynamic system 
$$
f(t,x,u) = D\xi(t,x)
$$
where $D\in\mathbb{R}^{n\times l}$ is known to us and $\xi$ not. This is called  *uncertain linear system*.

> Definition(Mathced uncertainty):
>
> In a uncertain linear system , if Im(D) $\subset$ Im(B), the system is described as matched uncertainty, while others with D is unmatched system

So we have our first property

> Theorem:
>
> The ideal sliding motion is insensitive to matched uncertainty.

Proof :
$$
\dot{x} = P_sAx+P_sD\xi(t,x),\forall t \geqslant t_s, St_s=0 \\
\exists R\in\mathbb{R}^{m\times l},s.t.\quad D=BR \\
P_sD = P_sBR = 0 \\
\dot{x} = P_sAx,\forall t \geqslant t_s, St_s=0
$$


Under matched uncertainty, we show the second property

> Theorem:
>
> The sliding motion under matched uncertainty is of reduced order and and its non zero eigenvectors os decided by sliding plane

Proof:

Consider $Sx(t) = 0, \forall t >t_s$   as linear equation, since S is row rull rank, there are (n-m) independent components in state variable, namely a reduction on order. For ideal sliding motion:

$A_{eq} = P_sA, SA_eq = 0\implies SA_{eq}v_i = 0 \implies \lambda_iSv_i = 0 \implies Sv_i = 0$



## Regular Form

Using QR transformation, we decompose B into 
$$
T_r B = [0\quad B_2]^T
$$
then the system is transformed into 
$$
\dot{x}_1(t) = A_{11}x_1(t)+A_{12}x_2(t) \\
\dot{x}_2(t) = A_{21}(t)+A_{22}x_2(t)+B_2u(t)
$$
This is called **regular form**, Then equation without input u is called **null space dynamics** and the other is called **range space dynamics**, 

## Reachability

For reachability, we mean that the sliding plane is (at least locally) an attractor, the most simple idea is to make 
$$
s\dot{s} < 0
$$
which is called reachability condition. However, this only gurantees converging to sliding plane asymptotically, So a stronger condition guaranteeing an ideal sliding motion is the $\eta$-reachability condition
$$
\dot{s}s \leqslant -\eta|s|
$$
