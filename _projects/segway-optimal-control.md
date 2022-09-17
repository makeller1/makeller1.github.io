---
classes: wide
excerpt: "Optimal control of the Segway."
header:
  teaser: "/assets/images/segway-project/segway-teaser.png"
---
<!-- Use the following script to see live updates of the jekyll local website -->
<head>
<script type="text/javascript" src="https://livejs.com/live.js"></script>
</head>

In this project I derived a model of a Segway and rider and implemented several optimal control methods in simulation. This work was part of the course Modern Control (a misnomer since the techniques are from the '60s). 

## System Description
<center>
<img width="225" src="/assets/images/segway-project/segway-and-rider.png" alt="segway and rider img">
</center>
<br>

The [Segway](https://en.wikipedia.org/wiki/Segway) is a two-wheeled personal transporter that is self-balancing thanks to its controller. In order to move, the rider shifts their center of mass in the direction they would like to travel and the controller responds to keep the rider level. The motors have a combined peak power of 3000 W.

The prototypical rider is modeled after [Paul Blart](https://en.wikipedia.org/wiki/Paul_Blart:_Mall_Cop) from the movie Mall Cop:

| Height | Mass | Inertia | Segway Mass |
|-------|--------|---------|:---------:|
| 1.68 $m$ | 100 $kg$ | 116 $kg \cdot m^2$ | 45 $kg$ |

## Lumped Mass Model

<img width="400" align="right" src="/assets/images/segway-project/lumped-mass-model.png" alt="lumped mass model img">
Several simplifying assumptions are made to derive a first order approximation of the system:

1. Travel occurs in a plane (no turning)
2. Small angle approximation
3. Rider is a single rigid body
4. Neglect motor dynamics and inertia of Segway
5. Frictionless surface

<br>
The resulting model is a 4<sup>th</sup> order system linearized about $\theta = 0$:

$$(m + M)\ddot x + \frac{L}2 m \ddot \theta = F$$

$$(J + \frac{mL^2}4) \ddot{\theta} + \frac{mL}2 \ddot{x} - \frac{mgL}2 \theta = 0$$

The model is put into state-space form for implementation in software, but is not shown since it is rather messy.

# Linear Quadratic Control

Linear quadratic (LQ) theory covers MIMO systems with linear dynamics and seeks to find an optimal controller with respect to a quadratic measure of performance. One of the most exciting results from the '60s was the proof that the optimal *closed-loop* LQ controller is a linear time-invariant state feedback controller, is asymptotically stable, and can be computed explicitly.

Assumptions on system:
1. Linear dynamics
2. Stabilizable and detectable
3. State feedback (all states are measured)

### Linear Quadratic Regulator

The linear quadratic regulator (LQR) in continuous time minimizes the following objective function:

$$ J(\boldsymbol x,\boldsymbol u) = \boldsymbol{x}^{\top}\boldsymbol{S}\boldsymbol{x} + \int_0^T\boldsymbol{x}^{\top}\boldsymbol{Q}\boldsymbol{x} \, + \boldsymbol{u}^{\top}\boldsymbol{R}\boldsymbol{u} \, \,dt$$ 

The weighting matrices $\boldsymbol{S}$ and $\boldsymbol{Q}$ are positive semi-definite and $\boldsymbol{R}$ is positive definite.

<p align="center">
<video src="/assets/images/segway-project/lqr.mp4" controls="controls" style="max-height: 250px;" type="video/mp4">
</video>
<br>
<em></em>
</p>

### Linear Quadratic Tracker

<p align="center">
<video src="/assets/images/segway-project/lqt-sinusoid.mp4" controls="controls" style="max-height: 250px;" type="video/mp4">
</video>
<br>
<em></em>
</p>

### State Observer

## LQ In Practice
LQ controllers can be difficult to tune because the weights $\boldsymbol{Q}, \boldsymbol R,$ and  $\boldsymbol S$ do not have an explicit relationship to the closed-loop poles. A careful balance must be struck (usually through much trial and error in simulation) between controlling the states and keeping the amount of control input reasonable. Perhaps the biggest shortcoming of the LQR variants is they cannot handle a nonlinearity that is almost always present in practice: **actuator saturation**. This was one of the main reasons driving development and use of the model predictive controller (MPC) beginning in the 70's and continuing to this day.