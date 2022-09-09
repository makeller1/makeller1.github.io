---
classes: wide
excerpt: "Whole-Body Control of the Stanford Pupper."
header:
  teaser: "/assets/images/pupper-teaser.png"
---
<!-- Use the following script to see live updates of the jekyll local website -->
<head>
<script type="text/javascript" src="https://livejs.com/live.js"></script>
</head>

This project started as a final project for [Alex](https://github.com/alexnavtt) and I in the course Decision and Control of Human-Centered Robots and continued on as a hobby for me. 

<p align="center">
<img width="840" src="/assets/images/pupper-standing.jpg">
</p>

The Pupper is an open-source torque-controlled robotic quadruped that comes packaged with a kinematic-based controller. The goal was to design a dynamic-based controller for the robot which incorporates much more information into control decisions for improved locomotion. Specifically, the controller is an Implicit Hierarchical Whole Body Controller (IHWBC) which enables:

1. Execution of multiple tasks simultaneously
2. Satisfaction of force and torque constraints
3. Compliant control
4. Real time implementation (1000 Hz +)

Furthermore, by modeling the dynamics, foot contact can be detected without tactile sensors.

The controller is written in C++ for efficiency and embedded deployment. An accurate dynamic model of the Pupper was created by modeling the assembly in SolidWorks and extracting inertial and kinematic properties of the links. The light-weight and embeddable [Rigid Body Dynamics Library](https://github.com/ORB-HD/rbdl-orb) is used for dynamic and kinematic calculations and [Gazebo](https://gazebosim.org) is used for testing and tuning in simulation. 

<!-- ### Hardware

The hip, shoulder, and elbow joints are composed of brushless DC motors that provide 12 actuated degrees of freedom. The motors provide feedback on the current and angular position through an incremental encoder. Low level current control runs on dedicated motor controllers at 1000 Hz. A 9 dof IMU provides feedback on angular position and velocity. A micro controller communicates with the motor controllers (I2C) and host computer (USB Serial), performs filtering and sensor fusion, and takes over high level control when faults are encountered. -->

## Implicit Hierarchical Whole Body Control
Whole Body Control describes a controller for an underactuated system that allocates degrees of freedom to more than one desired task (for instance, lift a foot and maintain balance are two tasks needed for locomotion). The Implicit Hierarchical Whole Body Controller (IHWBC) is a subset of WBC that defines a soft hierarchy among the tasks. Doing so ensures more important tasks like balance are executed when tasks conflict, which is quite common. 

A task is a desired body acceleration in the operational (body translation/rotation) or joint space:

$$\ddot x^d = \ddot x^{ff} + K_p (x^d - x) + K_d(\dot x^d - \dot x)$$

Given \(n\) tasks, the controller is posed as a convex optimization problem to find the optimal joint accelerations and reaction forces that minimize the $$L^2$$ norm of the task tracking error:

$$
\begin{aligned}
& \underset{F_r, \ddot q}{\text{minimize}}
& & \sum_i^n{\omega_i||J_i \ddot q + \dot{J_i}\dot q - \ddot{x}_i^d||^2 + \omega_f ||F_d-F_r||^2} 
\\
& \text{subject to}
& & \mathbf A \ddot q + b + g = ... J_c^\T
\end{aligned}
$$


Note: The controller performs an instantaneous optimization. Since the joint states are known, the non-linear terms in the equation of motion can be treated as constants. This allows a convex formulation crucial for real-time control.

### Footnotes

<!-- PURPOSE:

2 fold,
1. For prospective employers
2. For educating others and my future self

These goals can be compatible but there is a trade-off depending on the assumed audience. I will bias toward the second since I will be more motivated to help others / myself understand.

TLDR:
- Design of an Implicit Hierarchical Whole-Body Controller for model-based dynamics control
- Low-level and high-level control written in C++
- Involved embedded software
- Modeled dynamics with RBDL
- Simulated in Gazebo
- I used sophisticated and classic control algorithms
- Link code 

The Pupper is an open-source torque-controlled robotic quadruped that I wrote a dynamic controller for. 
-->