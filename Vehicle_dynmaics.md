# Background 

In reading papers about 4WIS control, I feel the need to read vehicle_dynamics.



# Day 1

## Basic system for research 

Yaw control->yaw stability control systems

Roll control->active rollover prevention systems

both yaw stability and roll over stability control -> integrated stability control

Commercial yaw stability control

Mostly differential-braking based systems. 

New: steer-by-wire and active torque distribution

Commercial Roll stability control

Active suspension system

Automated highway system (AHS) is a special case to discuss.

Adaptive cruise control (ACC)

Tilt control system.(Used for narrow vehicle)

Emission and fuel economy. 

Parallel/series hybrid

## Lateral dynamics

There are three main system in lateral dynamics: lane departure warning systems (LDWS), lane keeping systems (LKS) and yaw stability control systems. The last one is our main aim.

**Differential Braking systems** which utilize the ABS brake system on the vehicle to apply differential braking between the right and left wheels to control yaw moment.

**Steer-by-Wire systems** which modify the driver’s steering angle input and add a correction steering angle to the wheels

**Active Torque Distribution systems** which utilize active differentials and all wheel drive technology to independently control the drive torque distributed to each wheel and thus provide active control of both traction and yaw moment.(don't use)(can be used to sustantiate the advantage of steer-by-wire hahahaha)

### Terms for Lateral dynamics

Steering angle

center of gravity(c.g.)

wheelbase

slipangle

cornering stiffness

longitudinal tire stiffness

road bank angle

yaw rate

### Lateral dynamic models 

There are several basic model to be used for lateral dynamics:

1. Kinematic vehicle model(bycicle)

2. Dynamic vehicle model in terms of inertial lateral position and yaw

   angle

3. Dynamic vehicle model in terms of road-error variables

4. Dynamic vehicle model in terms of yaw rate and vehicle slip angle

#### Bycicle model(kinematic model)
An important assumption in bycicle model is that silp angle for both wheel equals zero, namely, there is no lateral force. This assumptions is reasonable in low speed. So what situation should we do? Actually there are situations where bycicle model is endowed with tire slip angle.

#### Dynamic vehicle model in terms of inertial lateral position and yaw angle

This model just use a polar coordinate-like system to derive the formula but takes lateral force into consideration.  Use samll angle approximation.

#### Dynamic vehicle model in terms of road-error variables

Used to construct LKS system. Assume longitudinal velocity is constant to make the system LTI.

#### Dynamic vehicle model in terms of yaw rate and vehicle slip angle

Use the angle of the front wheel to control slip angle and yaw rate, derived by Ackerman in 1997.

## Stability control

 The motivation for the development of yaw control systems comes from the fact that the behavior of the vehicle at the limits of adhesion is quite different from its nominal behavior. At the limits of adhesion, the slip angle is high and the sensitivity of yaw moment to changes in steering angle becomes highly reduced. At large slip angles, changing the steering angle produces very little change in the yaw rate of the vehicle. This is very different from the yaw rate behavior at low frequencies. 

On dry roads, vehicle maneuver- ability is lost at vehicle slip angles greater than ten degrees, while on packed snow, vehicle maneuverability is lost at slip angles as low as 4 degrees

The yaw control system addresses these issues by reducing the deviation of thevehicle behavior from its normal behavior on dry roads and by preventing the vehicle slip angle from becoming large.

### Differential braking system

The sensor set used by a differential braking system typically consists of four wheel speeds, a yaw rate sensor, a steering angle sensor, a lateral accelerometer and brake pressure sensors.

Typically the model for differential braking system is 7dof(the steering angle keep constant),  introduce slip ratio. 

#### Tire model and tire dynamics

In differential tire model, we use Dugoff tire model, it's propotional to the vertical force and nonlinear with slip ratio.  Tire dynamics is hard to reserach since we have no effective way to get the input torque and brake torque. The inherent assumption is that the rotational wheel dynamics are faster than the vehicle dynamics.





