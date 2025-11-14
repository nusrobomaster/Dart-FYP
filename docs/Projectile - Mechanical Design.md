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
<summary style="font-size: 1.5rem; font-weight: 450;"><strong> 2. Ground Rules </strong></summary>

Certain ground rules were established before beginning on the projectile designs. Here are the full design considerations, prior to beginning any design. Along the way, many other considerations were made which will be further elaborated below each projectile design respectively. 


* All projectile designs **SHALL NOT EXCEED 250mm x 250mm x 250mm**
* All projectile designs **SHALL BE BELOW 350g** under Solidworks evaluate tool, after specifying the material to be TPU 95D
* All projectile designs **will accommodate the DART TRIGGER** provided by RMOC
* All projectile designs will be designed to **NOT ROLL NATURALLY**, meaning the mechanical design will ensure the projectile does not roll without the need of corrective measures (such as active control surfaces)
* All projectiles will undergo Computational Fluid Dynamic analysis (CFD) on Solidworks, mainly for 2 reasons. 
First would be **straight line drag**, to minimize parasitic drag. From our literature review, a standard dart projectile has a linear velocity of 25m/s. We will analyze the cut plot of the projectile.
Second would be the **pressure distribution in the case of crosswinds**, which is to analyze if design can counter induced roll. We will induce a crosswind by setting x=25m/s, y=5m/s and z=5m/s to give a net speed of 25.98m/s, and we will analyze the pressure distribution across the projectile body.


</details>

<details markdown="1">
<summary style="font-size: 1.5rem; font-weight: 450;"><strong> 3. Center of Gravity vs Center of Pressure Analysis </strong></summary>

A baseline requirement for all projectile prototypes is passive static stability. This is achieved when the aerodynamic Center of Pressure (CP) is located behind the mass-based Center of Gravity (CG). This arrangement creates a "weather vane" effect, which is essential for a stable, predictable flight.
A two-step validation process will be applied to each of the seven design iterations to verify this condition.
The CG location for each prototype will be determined using a dual-method approach for accuracy:

The 3D-CAD model in Solidworks will be used to calculate the theoretical CG based on the specified material densities of all components. The calculation will then be physically validated by fabricating the prototype and performing a practical suspension (or "string test") to find its precise real-world balance point.
While the precise location of the CP is complex to derive analytically, a qualitative test will be performed on each prototype to confirm its inherent static stability. This test involves suspending the projectile from a string in front of a high-speed fan.
A successful validation occurs when the projectile immediately and stably orients itself into the wind and does not tumble. This behavior serves as a practical confirmation that the design possesses a positive static margin and that the CP is, as required, located behind the CG.




</details>


<details markdown="1">
<summary style="font-size: 1.5rem; font-weight: 450;"><strong> 4. 2 wings vs 3 wings vs 4 wings Analysis </strong></summary>

The selection of a four-fin "X" configuration over a two- or three-fin layout was a critical design choice driven by the demands of an active control system.
A two-fin, plane-like configuration (as explored in Prototype 2 "Glider") was fundamentally unsuitable. This design generates significant, uncontrolled aerodynamic lift, which destroys the predictable ballistic trajectory required for a 100mm target. Furthermore, it possesses no passive yaw stability, making it aerodynamically unstable as a projectile.
A three-fin "Y" configuration, while passively stable, was rejected due to control complexity. For an active system, a three-fin layout creates a **coupled control problem**. A pure "pitch" or "yaw" command cannot be executed by a single pair of fins. Instead, a complex **"control mixing" algorithm** is required to calculate the correct deflection angle for all three servos simultaneously. This adds unnecessary computational overhead and tuning difficulty.
To pitch straight up, you might only use the one bottom fin. To yaw left, you have to move both the top-left and top-right fins in a complex ratio.This is called **coupled control**. To move in any direction, we would have to run a "control mixing" algorithm that calculates the correct angle for all three fins. This is more complex, harder to tune, and can lead to unexpected cross-coupling effects.
With a **3-fin system**, a pure pitch-up maneuver relies on **only one fin** (the bottom one). This puts the entire load onto that single fin and servo, requiring it to be stronger (and likely heavier) to produce the same control force.

The four-fin "X" configuration was chosen because it provides a robust, **decoupled control system**. This layout provides two independent control axes:
With a **4-fin system**, any pitch or yaw command is handled by **two fins** working together. This distributes the aerodynamic load and the work across two servos, making the system more robust and responsive.
These axes are **decoupled**. When we command a "pitch up," we don't cause a yaw. This makes the control logic **dramatically simpler**. We can run two separate, simple PID controllers: one for pitch, one for yaw. It also distributes the aerodynamic load for any maneuver across two servos, increasing system responsiveness and robustness.



</details>


## 5. Projectile Design



## V2 GLIDER (Parallel Investigation)

As part of our first-principles methodology, we also investigated a radical alternative: the "Glider." This prototype was **not an iteration**, but a parallel test of a complex, UAV-like concept using a blended-wing-body and vectored thrust. The hypothesis was that we could actively "fly" the projectile to the target.
This investigation was a critical failure that provided immense value. CFD analysis revealed two fatal flaws. First, the design generated massive uncontrolled lift. This is catastrophic for a ballistic projectile, as it makes the trajectory unpredictable and impossible to aim. Second, this lift created enormous induced drag, in addition to the high base drag from its ducted-fan exit.
"Glider" was a conceptual dead end. It was heavy, complex, and its core "gliding" feature was the very thing that made it unusable. It provided definitive proof that an "over-engineered" approach was wrong and that our project must rely on a non-lifting, low-drag projectile configuration.


<details markdown="1">
<summary style="font-size: 1.5rem; font-weight: 450;"><strong> In-Depth Analysis on THORN </strong></summary>

## 1. Trajectory Failure: Uncontrolled Passive Lift
The goal of this project is to hit a 140mm x 140mm target, which requires a highly **predictable trajectory**. The "Glider," by design, is a high-lift body. Based on **Bernoulli's Principle**, the wings are shaped to create a pressure differential: lower pressure on top and higher pressure below. This pressure difference, L = ∆P . AWing   , creates an upward force. This lift is **passive and uncontrolled**. It causes the projectile to deviate from a predictable parabolic arc, likely entering an oscillating "phugoid" path. Any minor change in launch angle would be amplified by this lift, making consistent aiming impossible. 

## 2. Drag Failure: Induced Drag
The "Glider" also suffered from exceptionally high drag, far exceeding "Thorn." In addition to the large base drag from its blunt duct exit, it created a new, powerful drag component: induced drag.
Induced drag is the unavoidable aerodynamic "cost" of creating lift.
The magnitude of this drag (Di) is proportional to the square of the lift (L) it generates, as shown in the simplified formula:


$$D_i = \frac{L^2}{\frac{1}{2} \rho v^2 \pi b^2 e}$$

Where b is the wingspan and e is an efficiency factor. This shows that our high-lift "Glider" was inherently a high-drag design. This "over-engineered" prototype was a lesson in selecting the correct aerodynamic philosophy for the mission.


</details>


## V1 THORN

Prototype 1, "Thorn," was our baseline. A simple, passively-stable ballistic dart projectile. The goal was to establish a "control group"—the most basic, robust design to benchmark stability and manufacturing. Its cruciform ("+") fin configuration proved to be perfectly stable, as confirmed by CFD, which showed the fins correctly generated restoring forces to cancel out roll.
However, the CFD also revealed a fatal, high-impact flaw: critically high drag. The combination of a non-aerodynamic square body and a blunt, flat base created a massive, turbulent wake. This resulted in an exceptionally high drag coefficient (CD) that would demand immense launch energy and make the trajectory highly unpredictable.
"Thorn" proved that stability was easily achieved with inclusion of fins. It was rejected because it taught us the real challenge was managing drag. This forced our subsequent iterations to focus on aerodynamic efficiency and body-shape optimization.

<details markdown="1">
<summary style="font-size: 1.5rem; font-weight: 450;"><strong> In-Depth Analysis on THORN </strong></summary>

The "Thorn" prototype failed due to its high aerodynamic drag. Total drag (D) is the force that opposes the projectile's motion, and for a non-lifting body, it is dominated by parasitic drag ( DP). This is calculated using the master drag equation:

$$D = \frac{1}{2} \rho v^2 C_D A$$

Where:
 is the density of air
 v is the projectile's velocity
 A is the frontal reference area
 CD is the drag coefficient, which quantifies the shape's inefficiency
 Parasitic drag (DP) is itself a sum of two primary components: **skin friction drag** and **form drag** (also called pressure drag).

## Skin Friction Drag
This is the drag caused by the friction of the air "sticking" to the projectile's "wetted area" (its total outer surface). It is a function of the air's viscosity and the surface roughness. For a 3D-printed TPU body, which is relatively rough, this force is not negligible.

## Form Drag (Pressure Drag)
This was the **dominant and critical failure** of "Thorn." Form drag is caused by the projectile's shape, specifically the pressure difference between the high-pressure air at the front and the low-pressure air at the back.
* **Blunt Base & Flow Separation**: "Thorn's" flat base is the worst-case scenario for this. As the 25 m/s airflow passes the sharp rear edges, it cannot follow the contour and **separates** from the body. This creates a large, turbulent, low-pressure "wake" that "sucks" the projectile backward. This specific form drag, known as **base drag**, is the primary source of drag for a blunt-ended object.
* **Square Cross-Section**: The drag was severely worsened by the **square body**. Unlike a streamlined cylinder, the air flowing over the four sharp corners also separates, creating **asymmetric vortex shedding**. This not only adds to the drag but also creates unstable side-forces that would make the projectile's path erratic.
In summary, "Thorn's" high CD was a direct result of a shape that maximized form drag. It was rejected because its design actively promoted flow separation, creating a massive low-pressure wake that would make it unusable.



</details>


## V3 CROSSBLADE

"Crossblade" was our direct response to the failures of "Thorn." We applied the key lesson—that body shape is the dominant factor in drag—and replaced "Thorn's" unpredictable square body with a **streamlined, faceted-cylindrical fuselage.** We also transitioned from a "+" fin to an **"X" fin configuration**, not for stability (as both are stable), but for the practical advantages of **ground clearance** and better launch-rail integration.
CFD analysis confirmed this was a massive success. The airflow was clean, attached, and the chaotic drag from "Thorn" was eliminated. "Crossblade" was a **successful and viable passive dart.**
Its very success was its one limitation: as a passive projectile, it had no way to correct for minor errors. "Crossblade" proved we had mastered the stable, low-drag airframe, and it was the necessary justification to progress to an active control system.


<details markdown="1">
<summary style="font-size: 1.5rem; font-weight: 450;"><strong> In-Depth Analysis on THORN </strong></summary>

The "Crossblade" prototype was designed to solve the two primary drag problems identified in ***"Thorn"***, which was **form drag** and **base drag**.


## Form Drag Reduction (Body Shape)
The **square body** of "Thorn" was its critical flaw. Its four sharp corners caused **asymmetric vortex shedding**, an unstable and chaotic "flapping" of the wake. This not only created high drag but also applied random side-forces, making the projectile's path unpredictable.
"Crossblade" solves this by using a **faceted, near-cylindrical body**. This shape is nearly **axisymmetric**, which allows the airflow to remain attached to the body. This eliminates the chaotic vortex shedding, making the drag profile not only lower but, more importantly, **stable and predictable**.


## Base Drag Reduction (Boattail)
The second flaw in "Thorn" was its **high base drag** from a blunt, flat rear. This creates a large, low-pressure, turbulent wake that "sucks" the projectile backward.
"Crossblade" implements a **tapered "boattail"** at the rear. This simple geometric change has a massive aerodynamic effect.
The taper allows the airflow to **converge gradually** behind the projectile, keeping the flow "attached" for longer. This dramatically **reduces the size of the turbulent wake**. A smaller wake means the pressure at the base is higher (less suction). This reduced pressure differential between the front and back of the projectile results in a **significant reduction in base drag**.
This prototype was not, however, without new trade-offs. The design introduced two **mid-body sled guides** ("handles"). These are a necessary mechanical interface for the launcher but also create a new, albeit smaller, source of **parasitic drag**. This was deemed an acceptable engineering compromise.


While the "Crossblade" prototype proved a viable solution for a static (Level 1) target, the competition's advanced tiers introduce dynamic challenges. The following embedded videos illustrate the "Level 2" and "Level 3" targets, which move in complex, non-predictable patterns.
This requirement for **in-flight target interception** is the primary justification for progressing beyond a simple passive dart. A passive projectile's trajectory is fixed upon launch, making it incapable of hitting a target that has moved from its original position. Therefore, an **active projectile** with a guidance system and control surfaces is a fundamental necessity for success.



</details>

