---
classes: wide
excerpt: "Whole Body Control of the Stanford Pupper."
header:
  teaser: "/assets/images/pupper-project/pupper-standing.jpg"
---
<!-- Use the following script to see live updates of the jekyll local website -->
<head>
<script type="text/javascript" src="https://livejs.com/live.js"></script>
</head>

This project began as a final project for [Alex](https://github.com/alexnavtt) and I in the course Decision and Control of Human-Centered Robots and continued on as a hobby for me. [The repo for this project can be found on my github](https://github.com/makeller1/PupperWholeBodyControl).

<p align="center">
<img width="640" src="/assets/images/pupper-project/pupper-standing.jpg"><br>
<em>Pupper Quadruped</em>
</p>

The Pupper is an open-source torque-controlled robotic quadruped that comes packaged with a kinematic-based controller. The goal was to design a dynamic-based controller for the robot which incorporates much more information into control decisions for improved locomotion. Specifically, the controller is an Implicit Hierarchical Whole Body Controller (IHWBC) which enables:

1. Execution of multiple tasks simultaneously
2. Satisfaction of force and torque constraints
3. Compliant control
4. Real time implementation (1000 Hz)

Furthermore, by modeling the dynamics, foot contact can be detected without tactile sensors.

The controller is written in C++ for performance and embedded deployment. Light-weight libraries are used: [Rigid Body Dynamics Library](https://github.com/ORB-HD/rbdl-orb) for dynamic and kinematic calculations and [OSQP](https://osqp.org/) for solving the quadratic program. [Gazebo](https://gazebosim.org) is used for testing and tuning in simulation, and ROS is used for communication.

<!-- ### Hardware

The hip, shoulder, and elbow joints are composed of brushless DC motors that provide 12 actuated degrees of freedom. The motors provide feedback on the current and angular position through an incremental encoder. Low level current control runs on dedicated motor controllers at 1000 Hz. A 9 dof IMU provides feedback on angular position and velocity. A micro controller communicates with the motor controllers (I2C) and host computer (USB Serial), performs filtering and sensor fusion, and takes over high level control when faults are encountered. -->

# Whole Body Control
Whole Body Control (WBC) describes a controller for an underactuated system that allocates degrees of freedom to more than one desired task (for instance, lift a foot and maintain balance are two tasks needed for locomotion). The Implicit Hierarchical Whole Body Controller (IHWBC) is a type of WBC that defines a soft hierarchy among the tasks. Doing so ensures more important tasks like balance take priority when tasks conflict, which is quite common. 

A task is a desired body acceleration in the operational (body translation/rotation) or joint space:


$$\ddot{\textbf{x}}^d = \ddot{\textbf{x}}^{ff} + \textbf{K}_p (\textbf{x}^d - \textbf{x}) + \textbf{K}_d(\dot{\textbf{x}}^d - \dot{\textbf{x}})$$

Notice the task is calculated using feedback on the position error $(\textbf{x}^d - \textbf{x})$ and velocity error $(\dot{\textbf{x}}^d - \dot{\textbf{x}})$. Thus, we can track a desired position or velocity by tracking the desired acceleration. 

Given $n$ tasks, the controller is posed as a quadratic program to find the optimal joint accelerations and reaction forces that minimize the $L^2$ norm of the task tracking error:
<div>
$$\begin{aligned}
& \underset{\ddot{\textbf{q}}, \, \textbf{F}_r}{\text{minimize}}
& & \sum_i^n{\omega_i||\textbf{J}_i \ddot{\textbf{q}} + \dot{\textbf{J}_i}\dot{\textbf{q}} - \ddot{\textbf{x}}_i^d||^2 + \omega_f ||\textbf{F}_d-\textbf{F}_r||^2} 
\\
& \text{subject to}
& & \mathbf M \ddot{\textbf{q}} + \textbf{b} + \textbf{g} = \begin{pmatrix} \textbf{0}_{6x1} \\ \boldsymbol{\tau}^{cmd} \end{pmatrix} + \textbf{J}_c^\top \textbf{F}_r,
\\
& 
& & \textbf{U} \textbf{F}_r \ge \textbf{0},
\\ 
&
& & \boldsymbol{\tau}_{min} \le \boldsymbol{\tau}^{cmd} \le \boldsymbol{\tau}_{max}
\end{aligned}$$
</div>

Desired reaction forces $\textbf{F}_d$ are also tracked for smooth liftoff/touch down or otherwise if provided by a high level planner. The weights $\{\omega_i\}$ define the task priorities, but large tracking errors can make lower priority tasks take precedence unless the weights are seperated by several orders of magnitude. To prevent slipping, Coulomb friction cone constraints are linearized into [pyramid cones](https://scaron.info/robot-locomotion/friction-cones.html) represented by $\textbf{U} \textbf{F}_r \ge \textbf{0}$. 

The controller uses the *full* rigid body dynamics, so a compromise must be made for real-time control: the solution is an instantaneous optimization of one point in time. Doing so lets us treat the non-linear terms as constants and keep the problem convex for fast and guaranteed convergence. 


# Modeling

## Quadruped Model
<img width="400" align="right" src="/assets/images/pupper-project/solidworks-model.png" alt="SolidWorks Model">

Great care was taken to create the dynamic model of the Pupper because beyond the obvious benefits of an accurate model, it is challenging to validate it. Each part was weighed and its density calculated using the part's volume assuming a homogenous mass distribution. A homogeneous assumption is reasonable for the 3D printed parts because they are thin in the directions of low infill. For the other parts, like motors and PCBs, the assumption is justified by their small volumes. Because the links are 3D printed, the kinematic measurements are within the tolerance of the printers (~1%). The parts were assembled in SolidWorks and the inertial and kinematic properties of the links were extracted noting that SolidWorks uses a [positive product-of-inertia convention](https://www.mathworks.com/help/sm/ug/specify-custom-inertia.html).



Two models were then written into code, one for real-time dynamic and kinematic calculations, and another for simulation. The former was written in C++ using RBDL's modeling API. The latter was written in URDF format using the ROS package, [Xacro](http://wiki.ros.org/xacro). These models were validated against each other by comparing the measured reaction forces in simulation (Gazebo) with the optimal reaction forces calculated by the controller (WBC). The forces are nearly identical except for the initial transients caused by Gazebo's physics engine. 

<p align="center">
<img width="500" src="/assets/images/pupper-project/gazebo-vs-wbc.png" alt="Model Validation"><br>
<em>Validation of RBDL and Gazebo model matching</em>
</p>

## Motor Model

<p align="center">
<video src="/assets/images/pupper-project/motor-comparison.mp4" controls="controls" style="max-height: 500px;" type="video/mp4">
</video>
<br>
<em>Before compensation (left) and after compensation (right) </em>
</p>

The motors on the Pupper use a planetary gearbox with a 36:1 gear reduction ratio which keeps the motors compact and low-cost, but introduces inertia and friction that cannot be neglected. By building an accurate model of the motor, we can ensure the optimal torques determined by the controller are actually applied to the joints. Specifically, the motor friction is compensated with feed-forward control and the reflected inertia are added as independent terms to the mass matrix. 

<!-- ![constant torque response](/assets/images/pupper-project/rotor_inertia_identification.png){: .align-right width="50%"} -->

<img width="500" align="right" src="/assets/images/pupper-project/rotor_inertia_identification.png" alt="constant torque response">

One way to validate the model is by applying a constant torque and measuring the rotor's angular velocity. If friction is properly compensated, the velocity should be linear after the initial transients of the low-level current controller. This procedure was also used to estimate the reflected inertia as

$$I_{reflected} = \tau / \alpha$$ 

where $\alpha$ is the acceleration of the rotor (slope of linear regression). I found this estimate to be much more accurate than $I_{reflected} = 36^2 I_{rotor}$, primarily because of the difficulty of accurately measuring the inertia of the rotor and gears. 


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
- Advanced and classic control algorithms
- Link code 

-->