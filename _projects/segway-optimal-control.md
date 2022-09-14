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

In this project I derive a model of the [Segway](https://en.wikipedia.org/wiki/Segway) and implement several optimal control methods in simulation. This was my final project in the course Modern Control. 

## System Description
<center>
<img width="225" src="/assets/images/segway-project/segway-and-rider.png" alt="segway and rider img">
</center>
<br>
The Segway is a two-wheeled personal transporter that is self-balancing thanks to its controller. In order to move, the rider shifts their center of mass in the direction they would like to travel. 

The prototypical rider is modeled after [Paul Blart](https://en.wikipedia.org/wiki/Paul_Blart:_Mall_Cop) from the movie Mall Cop:
<center>

| Height | Mass | Inertia | Segway Mass |
|-------|--------|---------|:---------:|
| 1.68 $m$ | 100 $kg$ | 116 $kg \cdot m^2$ | 45 $kg$ |

</center>
## Lumped Mass Model

<img width="400" align="right" src="/assets/images/segway-project/lumped-mass-model.png" alt="lumped mass model img">
Several simplifying assumptions are made to derive a first order approximation of the system.
1. Small angle approximation
2. Rider is a single rigid body
3. Neglect motor dynamics and inertia of Segway
4. Frictionless surface

The resulting model is a 4th order linear system