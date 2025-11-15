---
title: Launcher - Mechanical Design
parent: Launcher Subsystem
nav_order: 1
layout: default
permalink: /launcher/mech_design
---
<!-- nav_order is for DROPDOWN NAVIGATION sequence <--> 

## Launcher Design

## 1. Foreground
The launcher is the sub-system responsible for imparting the required initial velocity to the projectile. The design of this sub-system is governed by two primary sets of constraints. First are the size and safety restrictions imposed by the RMOC rules. Second is the critical operational requirement from NUS Calibur Robotics for a propulsion system that delivers an exceptionally **consistent and reliable** launch velocity, as this is the most significant variable for ensuring target accuracy.


<details markdown="1">
<summary style="font-size: 1.3rem; font-weight: 450;"><strong> 3. Center of Gravity vs Center of Pressure Analysis </strong></summary>

picture

<br>
![projectile string test](/assets/images/shyam/stringtest.jpg)
{: .text-center}
<br>
<p align="center" class="small-text">
<em>Figure 5: String Test to locate Center Of Pressure.</em>
</p>


Passive static stability (Center of Pressure behind Center of Gravity) is a non-negotiable requirement for a predictable flight. This arrangement creates a "weather vane" effect, which is essential for a stable, predictable flight. We applied a two-part validation **to each and every prototype.**

* 1. Center of Gravity (CG) Determination: The CG was first calculated in Solidworks based on material densities. This theoretical value was then physically verified by fabricating the prototype and performing a suspension "string test" to find the real-world balance point.

* 2. Static Stability (CP vs. CG) Validation: A qualitative "fan test" was performed to confirm a positive static margin. Each prototype was suspended in front of a high-speed fan. A successful validation occurred when the projectile stably oriented itself into the wind, confirming the CP was correctly located behind the CG.

inspiration : https://www.youtube.com/watch?v=jikEHfFwBd8

</details>

