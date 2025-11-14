---
layout: default
title: Projectile - Mechanical Design
parent: Dart Projectile
nav_order: 1
permalink: /projectile/mech_design
---

## Dart Projectile

## 1. Foreground
The projectile is the primary payload of the DART system, and its design was a foundational focus of this project. The projectile's geometry and mass are critical system-level constraints that directly influence the design of the **feeder mechanism** and the **launcher**.
The project is governed by strict parameters from the competition organizer, RMOC, which define the maximum allowable size, weight, and power limits. However, the specific aerodynamic design of the projectile is unrestricted, requiring a rigorous development process involving iterative design and **Computational Fluid Dynamics (CFD) analysis.**

<br>
![Shenzhen Bay Sports Center Stadium](assets/images/shyam/shenzhenarena.jpg)
{: .text-center}
<br>
<p align="center" class="small-text">
<em>Figure 1: The Shenzhen Bay Sports Center Stadium, the competition venue.</em>
</p>

<br>
![Enclosed competition arena](assets/images/shyam/arena.jpg)
{: .text-center}
<br>
<p align="center" class="small-text">
<em>Figure 2: The enclosed competition arena, which has internal HVAC-induced crosswinds.</em>
</p>

A critical step before simulation was to establish the environmental base parameters. Misinterpreting these conditions, such as air density or wind speed, can invalidate CFD results.
The competition will be held at the Shenzhen Bay Sports Center Stadium, an enclosed, 48,210-square-meter arena. While NUS Calibur Robotics suggested this indoor location would have no wind, our team identified a significant risk of **internal crosswinds**. This was based on direct observation of large-scale **HVAC (air conditioning) units** during a 2024 site visit.
Therefore, despite the "enclosed" setting, a key design requirement was to **ensure projectile stability in the presence of crosswinds**, departing from the initial client assumption of a static-air environment.


<br>
![DART System testing area at NUS](assets/images/shyam/Darttesting.jpg)
{: .text-center}
<br>
<p align="center" class="small-text">
<em>Figure 3: Our DART System testing area at the NUS Engineering Auditorium.</em>
</p>

<br>
![Enclosed Multi Purpose Sports Hall](assets/images/shyam/mpshnus.jpg)
{: .text-center}
<br>
<p align="center" class="small-text">
<em>Figure 4: Enclosed Multi-Purpose Sports Hall used for additional testing.</em>
</p>

For the design of the projectile, we decided to adopt a **first-principles design methodology**. This is a Problem-solving approach that involves breaking down a complex issue into its most fundamental truths to build innovative solutions from the ground up.
We intentionally began with simple, passive prototypes. This was critical for us to learn the fundamental pain points of stability and drag, before we could justify adding the complexity of an active system. Our first prototypes were a rapid exploration of basic concepts.


### Iteration Path:

* **Passive projectile design:**
    * V1 (Thorn) → V3(Crossblade) ✅
* **Parallel investigation (Conceptual Detour):**
    * V2 (Glider) ❌
* **Active projectile design:**
    * V4(Xwing) → V5(Xwing 2) → V6(Trident) → V7(Hunter) ✅

<details markdown="1">
<summary style="font-size: 2rem; font-weight: 600;"><strong> 2. Ground Rules </strong></summary>

Certain ground rules were established before beginning on the projectile designs. Here are the full design considerations, prior to beginning any design. Along the way, many other considerations were made which will be further elaborated below each projectile design respectively. 


* All projectile designs **SHALL NOT EXCEED 250mm x 250mm x 250mm**
* All projectile designs **SHALL BE BELOW 350g** under Solidworks evaluate tool, after specifying the material to be TPU 95D
* All projectile designs **will accommodate the DART TRIGGER** provided by RMOC
* All projectile designs will be designed to **NOT ROLL NATURALLY**, meaning the mechanical design will ensure the projectile does not roll without the need of corrective measures (such as active control surfaces)
* All projectiles will undergo Computational Fluid Dynamic analysis (CFD) on Solidworks, mainly for 2 reasons. 
First would be **straight line drag**, to minimize parasitic drag. From our literature review, a standard dart projectile has a linear velocity of 25m/s. We will analyze the cut plot of the projectile.
Second would be the **pressure distribution in the case of crosswinds**, which is to analyze if design can counter induced roll. We will induce a crosswind by setting x=25m/s, y=5m/s and z=5m/s to give a net speed of 25.98m/s, and we will analyze the pressure distribution across the projectile body.


</details>