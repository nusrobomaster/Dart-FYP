---
title: 2 Mechanical Design
parent: Projectile
grand_parent: Dart System
nav_order: 1
layout: default
permalink: /dart-system/projectile/mechanical
has_toc: true
---

<!-- existing content -->

## Dart Projectile

## 1. Foreground
The projectile is the primary payload of the DART system, and its design was a foundational focus of this project. The projectile's geometry and mass are critical system-level constraints that directly influence the design of the **feeder mechanism** and the **launcher**.
The project is governed by strict parameters from the competition organizer, RMOC, which define the maximum allowable size, weight, and power limits. However, the specific aerodynamic design of the projectile is unrestricted, requiring a rigorous development process involving iterative design and **Computational Fluid Dynamics (CFD) analysis.**

<br>
![Shenzhen Bay Sports Center Stadium]({{ '/assets/images/shyam/shenzhenarena.jpg' | relative_url }})
{: .text-center}
<br>
<p align="center" class="small-text">
<em>Figure 1: The Shenzhen Bay Sports Center Stadium, the competition venue.</em>
</p>

<br>
![Enclosed competition arena]({{ '/assets/images/shyam/arena.jpg' | relative_url }})
{: .text-center}
<br>
<p align="center" class="small-text">
<em>Figure 2: The enclosed competition arena, which has internal HVAC-induced crosswinds.</em>
</p>


A critical step before simulation was to establish the environmental base parameters. Misinterpreting these conditions, such as air density or wind speed, can invalidate CFD results.
The competition will be held at the Shenzhen Bay Sports Center Stadium, an enclosed, 48,210-square-meter arena. While NUS Calibur Robotics suggested this indoor location would have no wind, our team identified a significant risk of **internal crosswinds**. This was based on direct observation of large-scale **HVAC (air conditioning) units** during a 2024 site visit.
Therefore, despite the "enclosed" setting, a key design requirement was to **ensure projectile stability in the presence of crosswinds**, departing from the initial client assumption of a static-air environment.


<br>
![DART System testing area at NUS]({{ '/assets/images/shyam/Darttesting.png' | relative_url }})
{: .text-center}
<br>
<p align="center" class="small-text">
<em>Figure 3: Our DART System testing area at the NUS Engineering Auditorium.</em>
</p>


<br>
![Enclosed Multi Purpose Sports Hall]({{ '/assets/images/shyam/mpshnus.jpg' | relative_url }})
{: .text-center}
<br>
<p align="center" class="small-text">
<em>Figure 4: Enclosed Multi-Purpose Sports Hall used for additional testing.</em>
</p>


For the design of the projectile, we decided to adopt a **first-principles design methodology**. This is a Problem-solving approach that involves breaking down a complex issue into its most fundamental truths to build innovative solutions from the ground up.
We intentionally began with simple, passive prototypes. This was critical for us to learn the fundamental pain points of stability and drag, before we could justify adding the complexity of an active system. Our first prototypes were a rapid exploration of basic concepts.


---

### Iteration Path:

* **Passive projectile design:**
     V1 (Thorn) → V3(Crossblade) ✅
* **Parallel investigation (Conceptual Detour):**
     V2 (Glider) ❌
* **Active projectile design:**
     V4(Xwing) → V5(Xwing 2) → V6(Trident) → V7(Hunter) ✅

---

<details markdown="1">
<summary style="font-size: 1.3rem; font-weight: 450; color: #0059b3;"><strong> 2. Ground Rules </strong></summary>

Certain ground rules were established before beginning on the projectile designs. Here are the full design considerations, prior to beginning any design. Along the way, many other considerations were made which will be further elaborated below each projectile design respectively. 

* All projectile designs **SHALL NOT EXCEED 250mm x 250mm x 250mm**
* All projectile designs **SHALL BE BELOW 350g** under Solidworks evaluate tool, after specifying the material to be TPU 95D
* All projectile designs **will accommodate the DART TRIGGER** provided by RMOC
* All projectile designs will be designed to **NOT ROLL NATURALLY**, meaning the mechanical design will ensure the projectile does not roll without the need of corrective measures (such as active control surfaces)
* All projectiles will undergo Computational Fluid Dynamic analysis (CFD) on Solidworks, mainly for 2 reasons. 

First would be **straight line drag**, to minimize parasitic drag. From our literature review, a standard dart projectile has a linear velocity of 25m/s. We will analyze the cut plot of the projectile.

Second would be the **pressure distribution in the case of crosswinds**, which is to analyze if design can counter induced roll. We will induce a crosswind by setting x=25m/s, y=5m/s and z=5m/s to give a net speed of 25.98m/s, and we will analyze the pressure distribution across the projectile body.


</details>


---


<details markdown="1">
<summary style="font-size: 1.3rem; font-weight: 450; color: #0059b3;"><strong> 3. Center of Gravity vs Center of Pressure Analysis </strong></summary>

picture

<br>
![projectile string test]({{ site.baseurl }}/assets/images/shyam/stringtest.jpg)
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


---


<details markdown="1">
<summary style="font-size: 1.3rem; font-weight: 450; color: #0059b3;"><strong> 4. TWO wings vs THREE wings vs FOUR wings Analysis </strong></summary>

<br>
![gliderpressure1]({{ site.baseurl }}/assets/images/shyam/2fin.png)
{: .text-center}
<br>
<p align="center" class="small-text">
</p>

The selection of a 4-fin "X" configuration over 2- or 3-fin layouts was a critical choice driven by the demands of our active control system.

**2-Fin Rejection:** A two-fin, plane-like configuration ("Glider") was fundamentally unsuitable. It generates significant uncontrolled lift, which destroys the predictable ballistic trajectory, and it lacks the passive yaw stability required for a projectile.

**3-Fin Rejection:** A three-fin "Y" configuration, while passively stable, was rejected due to control complexity. It creates a coupled control problem, where any pure pitch or yaw maneuver requires a complex "control mixing" algorithm to coordinate all three servos. This is difficult to tune and places the entire aerodynamic load for some maneuvers onto a single servo, requiring heavier hardware.

**4-Fin Selection:** The 4-fin "X" layout was chosen because it provides a robust, decoupled control system. This configuration creates two independent axes for pitch and yaw, allowing for simple, independent PID controllers. Critically, this design distributes the aerodynamic load for any maneuver across two servos, increasing system responsiveness and robustness.

</details>


---


## 5. Projectile Design


## V2 GLIDER (Parallel Investigation)


<br>
![Glider Real]({{ '/assets/images/shyam/gliderreal.jpg' | relative_url }})
{: .text-center}
<br>
<p align="center" class="small-text">
<em>Figure 6: 3D printed scale model of GLIDER</em>
</p>

<br>
![Glider CAD]({{ '/assets/images/shyam/glidercad.png' | relative_url }})
{: .text-center}
<br>
<p align="center" class="small-text">
<em>Figure 7: 3D design of GLIDER.</em>
</p>


As part of our first-principles methodology, we conducted a parallel investigation with the "Glider" prototype. This complex, UAV-like concept used a blended-wing-body and vectored thrust to test the hypothesis of actively "flying" the projectile to the target.

The investigation was a critical failure that provided immense value. CFD analysis revealed two fatal flaws:

**Uncontrolled Lift**: The design generated massive passive lift, which is catastrophic for a ballistic projectile as it makes the trajectory unpredictable and un-aimable.

**Excessive Drag**: This lift, in turn, created enormous induced drag, in addition to high base drag from the ducted-fan exit.

"Glider" was a conceptual dead end. It was heavy, complex, and its core "gliding" feature made it unusable. This test provided definitive proof that an "over-engineered" approach was wrong and that our project must rely on a non-lifting, low-drag projectile configuration.


<details markdown="1">
<summary style="font-size: 1.3rem; font-weight: 450; color: #0059b3; "><strong> In-Depth Analysis on GLIDER (Click to view) </strong></summary>

<br>
![gliderpressure1]({{ site.baseurl }}/assets/images/shyam/gliderpressure1.png)
{: .text-center}
<br>
<p align="center" class="small-text">
<em>Figure 8: Surface Plot (pressure) on GLIDER.</em>
</p>

<br>
![gliderpressure2]({{ site.baseurl }}/assets/images/shyam/gliderpressure2.png)
{: .text-center}
<br>
<p align="center" class="small-text">
<em>Figure 9: Surface Plot (pressure) on GLIDER.</em>
</p>


## 1. Trajectory Failure: Uncontrolled Passive Lift
The goal of this project is to hit a 140mm x 140mm target, which requires a highly **predictable trajectory**. The "Glider," by design, is a high-lift body. Based on **Bernoulli's Principle**, the wings are shaped to create a pressure differential: lower pressure on top and higher pressure below. This pressure difference, L = ∆P . AWing   , creates an upward force. This lift is **passive and uncontrolled**. It causes the projectile to deviate from a predictable parabolic arc, likely entering an oscillating "phugoid" path. Any minor change in launch angle would be amplified by this lift, making consistent aiming impossible. 


<br>
![gliderdrag]({{ site.baseurl }}/assets/images/shyam/gliderdrag.png)
{: .text-center}
<br>
<p align="center" class="small-text">
<em>Figure 10: Cut Plot (Velocity Drag) on GLIDER.</em>
</p>

## 2. Drag Failure: Induced Drag
The "Glider" also suffered from exceptionally high drag, far exceeding "Thorn." In addition to the large base drag from its blunt duct exit, it created a new, powerful drag component: induced drag.
Induced drag is the unavoidable aerodynamic "cost" of creating lift.
The magnitude of this drag (Di) is proportional to the square of the lift (L) it generates, as shown in the simplified formula:


$$D_i = \frac{L^2}{\frac{1}{2} \rho v^2 \pi b^2 e}$$

Where b is the wingspan and e is an efficiency factor. This shows that our high-lift "Glider" was inherently a high-drag design. This "over-engineered" prototype was a lesson in selecting the correct aerodynamic philosophy for the mission.


</details>

---

## V1 THORN

<br>
![Thorn Real]({{ '/assets/images/shyam/thornreal.jpg' | relative_url }})
{: .text-center}
<br>
<p align="center" class="small-text">
<em>Figure 11: 3D printed scale model of THORN</em>
</p>

<br>
![Thorn CAD]({{ '/assets/images/shyam/thorncad.png' | relative_url }})
{: .text-center}
<br>
<p align="center" class="small-text">
<em>Figure 12: 3D design of THORN.</em>
</p>


Prototype 1, "Thorn," was our baseline. A simple, passively-stable ballistic dart projectile. The goal was to establish a "control group". The most basic, robust design to benchmark stability and manufacturing. Its cruciform ("+") fin configuration proved to be perfectly stable, as **confirmed by CFD**, which showed the **fins correctly generated restoring forces to cancel out roll.**

However, the CFD also revealed a fatal, high-impact flaw: **critically high drag**. The combination of a non-aerodynamic square body and a blunt, flat base created a massive, turbulent wake. This resulted in an exceptionally high drag coefficient (CD) that would demand immense launch energy and make the trajectory highly unpredictable.

"Thorn" proved that stability was easily achieved with inclusion of fins. It was rejected because the real challenge was managing drag. This forced our subsequent iterations to **focus on aerodynamic efficiency** and **body-shape optimization.**

<details markdown="1">
<summary style="font-size: 1.3rem; font-weight: 450; color: #0059b3;"><strong> In-Depth Analysis on THORN (Click to view) </strong></summary>

<br>
![thorndrag]({{ site.baseurl }}/assets/images/shyam/thorndrag.png)
{: .text-center}
<br>
<p align="center" class="small-text">
<em>Figure 13: Cut Plot (Velocity Drag) on THORN.</em>
</p>

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

<br>
![gliderdrag]({{ site.baseurl }}/assets/images/shyam/thornwind.jpg)
{: .text-center}
<br>
<p align="center" class="small-text">
<em>Figure 14: Surface Plot (pressure) on THORN.</em>
</p>
<br>
![gliderdrag]({{ site.baseurl }}/assets/images/shyam/thornpressure2.png)
{: .text-center}
<br>
<p align="center" class="small-text">
<em>Figure 15: Surface Plot (pressure) on THORN.</em>
</p>


</details>

---

## V3 CROSSBLADE

<br>
![Crossblade real]({{ '/assets/images/shyam/crossbladereal.jpg' | relative_url }})
{: .text-center}
<br>
<p align="center" class="small-text">
<em>Figure 16: 3D printed scale model of CROSSBLADE</em>
</p>

<br>
![Crossblade CAD]({{ '/assets/images/shyam/crossbladecad.png' | relative_url }})
{: .text-center}
<br>
<p align="center" class="small-text">
<em>Figure 17: 3D design of CROSSBLADE.</em>
</p>


"Crossblade" was our direct response to the failures of "Thorn." We applied the key lesson—that body shape is the dominant factor in drag—and replaced "Thorn's" unpredictable square body with a **streamlined, faceted-cylindrical fuselage.** We also transitioned from a "+" fin to an **"X" fin configuration**, not for stability (as both are stable), but for the practical advantages of **ground clearance** and better launch-rail integration.
CFD analysis confirmed this was a massive success. The airflow was clean, attached, and the chaotic drag from "Thorn" was eliminated. "Crossblade" was a **successful and viable passive dart.**
Its very success was its one limitation: as a passive projectile, it had no way to correct for minor errors. "Crossblade" proved we had mastered the stable, low-drag airframe, and it was the necessary justification to progress to an active control system.


<details markdown="1">
<summary style="font-size: 1.3rem; font-weight: 450; color: #0059b3;"><strong> In-Depth Analysis on CROSSBLADE (Click to view)</strong></summary>

The "Crossblade" prototype was designed to solve the two primary drag problems identified in ***"Thorn"***, which was **form drag** and **base drag**.

<br>
![crossbladedrag]({{ site.baseurl }}/assets/images/shyam/crossbladedrag.png)
{: .text-center}
<br>
<p align="center" class="small-text">
<em>Figure 18: Cut Plot (Velocity Drag) on CROSSBLADE.</em>
</p>




## Form Drag Reduction (Body Shape)
The **square body** of "Thorn" was its critical flaw. Its four sharp corners caused **asymmetric vortex shedding**, an unstable and chaotic "flapping" of the wake. This not only created high drag but also applied random side-forces, making the projectile's path unpredictable.
"Crossblade" solves this by using a **faceted, near-cylindrical body**. This shape is nearly **axisymmetric**, which allows the airflow to remain attached to the body. This eliminates the chaotic vortex shedding, making the drag profile not only lower but, more importantly, **stable and predictable**.


## Base Drag Reduction (Boattail)
The second flaw in "Thorn" was its **high base drag** from a blunt, flat rear. This creates a large, low-pressure, turbulent wake that "sucks" the projectile backward.
"Crossblade" implements a **tapered "boattail"** at the rear. This simple geometric change has a massive aerodynamic effect.
The taper allows the airflow to **converge gradually** behind the projectile, keeping the flow "attached" for longer. This dramatically **reduces the size of the turbulent wake**. A smaller wake means the pressure at the base is higher (less suction). This reduced pressure differential between the front and back of the projectile results in a **significant reduction in base drag**.
This prototype was not, however, without new trade-offs. The design introduced two **mid-body sled guides** ("handles"). These are a necessary mechanical interface for the launcher but also create a new, albeit smaller, source of **parasitic drag**. This was deemed an acceptable engineering compromise.

<br>
![tpucrossblade]({{ site.baseurl }}/assets/images/shyam/tpucrossblade.jpg)
{: .text-center}
<br>
<p align="center" class="small-text">
<em>Figure 19: TPU95A CROSSBLADE for testing.</em>
</p>

While the "Crossblade" prototype proved a viable solution for a static (Level 1) target, the competition's advanced tiers introduce dynamic challenges. The following embedded videos illustrate the "Level 2" and "Level 3" targets, which move in complex, non-predictable patterns.
This requirement for **in-flight target interception** is the primary justification for progressing beyond a simple passive dart. A passive projectile's trajectory is fixed upon launch, making it incapable of hitting a target that has moved from its original position. Therefore, an **active projectile** with a guidance system and control surfaces is a fundamental necessity for success.

<br>
![crossblade pressure]({{ site.baseurl }}/assets/images/shyam/crossbladepressure.png)
{: .text-center}
<br>
<p align="center" class="small-text">
<em>Figure 20: Surface Plot (pressure) on CROSSBLADE.</em>
</p>


</details>

---

## V4 X-WING (Active Dart) 
*credit : Dalian University of Technology, open source design*


<br>
![xwing real]({{ '/assets/images/shyam/XWingreal.jpg' | relative_url }})
{: .text-center}
<br>
<p align="center" class="small-text">
<em>Figure 21: 3D printed scale model of X WING</em>
</p>

<br>
![xwing CAD]({{ '/assets/images/shyam/xwingcad.png' | relative_url }})
{: .text-center}
<br>
<p align="center" class="small-text">
<em>Figure 22: 3D design of X WING.</em>
</p>


Having mastered a passive dart, we next investigated the "active control" meta by analyzing an open-source competitor design: the "X-WING." This prototype added **three forward canards** and an internal ducted fan. Our analysis proved this popular concept to be a **"technical trap."**
CFD confirmed **massive parasitic drag** from all the exposed hinges and ducts. More critically, the 1-second flight time renders the fan **"dead weight"**, as it can't spin up. The fatal flaw, however, was **canard-vortex interaction**: the front canards shed "dirty air" that caused a severe, uncommanded **roll instability** at the tail. "X-WING" was a crucial failure that taught us to reject canard-based designs entirely.



<details markdown="1">
<summary style="font-size: 1.3rem; font-weight: 450; color: #0059b3;"><strong> In-Depth Analysis on XWing (Click to view) </strong></summary>

<br>
![xwingdrag]({{ site.baseurl }}/assets/images/shyam/xwingdrag.png)
{: .text-center}
<br>
<p align="center" class="small-text">
<em>Figure 23: Cut Plot (Velocity Drag) on X WING.</em>
</p>

<br>
![xwingpressure]({{ site.baseurl }}/assets/images/shyam/xwingpressure.png)
{: .text-center}
<br>
<p align="center" class="small-text">
<em>Figure 24: Surface Plot (pressure) on X WING.</em>
</p>




The rejection of "X-WING" was based on its three fundamental flaws, the most critical of which was a self-induced instability.

<br>
![canardroll]({{ site.baseurl }}/assets/images/shyam/canardroll.jpg)
{: .text-center}
<br>
<p align="center" class="small-text">
<em>Figure 25: Canard induced roll illustration.</em>
</p>
<br>
![canardroll]({{ site.baseurl }}/assets/images/shyam/roll.png)
{: .text-center}
<br>
<p align="center" class="small-text">
<em>Figure 26: Canard induced roll illustration.</em>
</p>

## 1. Aerodynamic Instability (Canard-Vortex Interaction)
Canards are control surfaces placed forward of the Center of Gravity (CG). To steer (e.g., pitch up), they generate a small lift force.
An unavoidable byproduct of any fin generating lift is the creation of powerful wingtip vortices.
These high-energy, swirling tubes of air, known as "dirty air", travel downstream and pass directly over the rear stabilizing fins. This creates a massive, asymmetric pressure loading on the tail, inducing a strong, uncommanded roll moment. The projectile is actively destabilizing itself, making precision control impossible. 
This is the "fatal flaw" that makes the design unviable.
longitudinal (pitch) stability formula. 
Static Stability Margin (CMa) must be negative for the projectile to be stable.

$$C_{M_{\alpha}} = \frac{dC_M}{d\alpha}$$


For a canard-based design, this stability is a balancing act between two main components:
* **The Canard (Destabilizing)**: It's a lifting surface in front of the Center of Gravity (CG). When the projectile pitches up, the canard lifts more, which creates a moment that pitches it even further up. This is **unstable.**
* **The Main Wing (Stabilizing)**: It's a lifting surface behind the CG. When the projectile pitches up, the main wing lifts more, creating a moment that pitches the nose back down. This is **stable.**

For the total projectile to be stable, the **stabilizing effect of the main wing must be stronger than the destabilizing effect of the canard.**
For a canard design to be stable, the **main wing area** (SW) must be proportionally much larger than the **canard area** (SC). This ensures the main wing's stabilizing moment (which is a function of its area and its distance from the CG) can overcome the canard's destabilizing moment.
However, the 250x250x150mm competition box severely **limited our maximum main wing area** (SW). To get any meaningful control, the canards (SC) still had to be a certain size. This resulted in a **Canard-to-Wing Area Ratio (SC / SW) that was far too high.**
The destabilizing effect of the "proportionally large" canards simply overpowered the small main wings. This made the projectile inherently unstable in pitch (CM> 0), causing it to tumble and making it uncontrollable. This pitch instability, combined with the separate problem of canard-vortex induced roll, made the entire concept unviable.


## 2. Non-Viable Propulsion and Control
The design included an Electro-Ducted Fan (EDF) and a thrust-vectoring system. This system is non-viable for two reasons:
* **Spool-Up Time**: An EDF has a significant "spool-up" time (0.5-1.0s) to reach its peak RPM and produce effective thrust. Given the projectile's total flight time is approximately one second, the fan would never reach a usable thrust level, rendering it (and the control system) dead weight.
Mass & Complexity: The fan, motor, and extra servos add significant mass, pushing the design to the 350g limit.
* **Excessive Parasitic Drag**
Unlike the clean "Crossblade" (P3), the "X-WING" is aerodynamically "dirty." Every exposed servo hinge, control horn, and the open side-ducts act as "parasites" that disrupt the airflow. Each of these components creates its own small, turbulent wake, and their cumulative effect results in an exceptionally high **parasitic drag** (DP) and a poor CD.

</details>

---

## V5 X-WING 2 (Active Dart) 
*custom modification from Dalian University of Technology, open source design*


<br>
![xwing2real]({{ '/assets/images/shyam/XWing2real.jpg' | relative_url }})
{: .text-center}
<br>
<p align="center" class="small-text">
<em>Figure 27: 3D printed scale model of XWing 2</em>
</p>

<br>
![XWing2 CAD]({{ '/assets/images/shyam/xwing2cad.png' | relative_url }})
{: .text-center}
<br>
<p align="center" class="small-text">
<em>Figure 28: 3D design of XWing 2.</em>
</p>


Prototype 5 was a final attempt to salvage the canard concept from "X-WING" (P4). We correctly identified a major structural flaw—body flex—and added a carbon fiber "spine" to create a rigid airframe. However, our aerodynamic "fix" (a 2-canard layout) failed to solve the core canard-vortex instability.
This prototype was the definitive "dead end." It taught us a crucial lesson, which was confirmed by external data from top teams: the entire canard-and-fan concept is overly complex, fragile, and aerodynamically flawed. This prototype was the final justification for abandoning canards entirely and pivoting to a simpler, more robust rear-fin control system.


<details markdown="1">
<summary style="font-size: 1.3rem; font-weight: 450; color: #0059b3;"><strong> In-Depth Analysis on XWing 2 (Click to view) </strong></summary>

<br>
![xwingdrag]({{ site.baseurl }}/assets/images/shyam/xwing2drag.png)
{: .text-center}
<br>
<p align="center" class="small-text">
<em>Figure 29: Cut Plot (Velocity Drag) on X WING2.</em>
</p>


<br>
![xwingdrag]({{ site.baseurl }}/assets/images/shyam/xwing2pressure.png)
{: .text-center}
<br>
<p align="center" class="small-text">
<em>Figure 30: Surface Plot (pressure) on X WING2.</em>
</p>


"V5 X-WING 2" was a direct iteration on P4, designed to solve two distinct problems.

## 1. The Structural Problem: Wing Misalignment
Based on supervisor feedback, we identified that the uncoupled TPU wings would suffer from **differential flex** in flight. The front X-wings and rear X-wings, being separate components, would flex independently under aerodynamic load. This is a critical control failure, as it creates **aerodynamic phase lag:**
* The canards at the front would actuate, slightly twisting their wing structure.
* This flex would not be transmitted uniformly to the rear stabilizing fins.
* This would **de-synchronize** the control surfaces from the stabilization surfaces, leading to an unpredictable response and making the projectile uncontrollable.

**The Solution**: Thin **carbon fiber splines** were added, connecting the outer tips of the front wings to the tips of the corresponding rear wings. These splines act as a torsional brace for the entire aerodynamic assembly. They rigidly couple the front and rear wing sets, forcing them to move as a single, coordinated unit. This solution massively increased the airframe's torsional stiffness, prevented wing flutter, and ensured the rear fins remained perfectly aligned with the front canards.

## 2. The Aerodynamic Problem: Roll Instability
We attempted to solve P4's roll instability by removing the bottom canard, creating a **2-canard configuration.** This "fix" was based on a flawed premise.
* The roll instability in P4 was caused by asymmetric vortex shedding.
* A 2-canard layout is symmetric from the start.
* This design does not solve the fundamental problem of canard-vortex interaction ("dirty air"). CFD analysis confirmed that significant, complex aerodynamic forces were still present.

## 3. Rejection and Final Validation

This prototype was rejected. While the carbon fiber splines were an excellent solution to a complex structural problem (wing flex), this fix was applied to a fundamentally flawed aerodynamic concept. The design remained heavy, high-drag, and unstable.
This conclusion was definitively confirmed by **external data** from the 2025 RMUC competition. Our team learned from the Dalian University of Technology (DUT) DART team that their similar canard-and-fan design achieved only **80% accuracy**, and did not make it into the top 10 performing DART teams. The top 10 teams (95%+ accuracy) used **simpler designs without canards or fans.****
The DUT team confirmed the added complexity and mass were **not justified by performance**. To stay under the 350g limit, they had to thin the projectile walls, leading to **high breakage rates** on impact. This real-world data provided the final evidence that the active canard/fan concept is a "dead end."
<br>
![results]({{ site.baseurl }}/assets/images/shyam/projresult.png)
{: .text-center}
<br>
<p align="center" class="small-text">
<em>Figure 31: 2025 RMUC competition Score.</em>
</p>



</details>
---

## V6 TRIDENT (Active Dart) 


<br>
![tridet Real]({{ '/assets/images/shyam/Tridentreal.jpg' | relative_url }})
{: .text-center}
<br>
<p align="center" class="small-text">
<em>Figure 32: 3D printed scale model of TRIDENT</em>
</p>

<br>
![trident CAD]({{ '/assets/images/shyam/tridentcad.png' | relative_url }})
{: .text-center}
<br>
<p align="center" class="small-text">
<em>Figure 33: 3D design of TRIDENT.</em>
</p>

<div style="text-align: center;">
<iframe width="560" height="315" src="https://www.youtube.com/embed/RYEWWLrXpE4?si=dgrxcvLTP-wGlQiI" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
</div>

<p class="figure-caption" style="text-align:center;">
Media 01: Video of Tridet Split-Up (New Segmented Design)
</p>

Prototype 6, "Trident," marks our critical pivot. Based on competitor data and the failures of P1-P5, we **abandoned the flawed canard concept** and adopted the correct philosophy: **rear-fin active control.**

This prototype was the **first to integrate the full electronics payload** and featured an **innovative modular "cassette" design for fast serviceability**. However, this "proof-of-concept" was a mechanical failure.

The long tie-rod linkages (from mid-body servos) created severe control slop and aerodynamic flutter. Furthermore, large servo cutouts and a "zip-tie" assembly method made the airframe fatally fragile and flexible.
"Trident" was a success in one way: it proved our philosophy was right, but our execution was wrong. It taught us the final, critical lesson: the control system must be direct-drive and the airframe must be structurally rigid.



<details markdown="1">
<summary style="font-size: 1.3rem; font-weight: 450; color: #0059b3;"><strong> In-Depth Analysis on TRIDENT (Click to view) </strong></summary>

"Trident" was a conceptual breakthrough. By moving the control surfaces to the rear, we use "clean," undisturbed air, eliminating the canard-vortex interaction ("dirty air") that caused the roll instability in P4 and P5. However, the implementation of this new philosophy revealed two major categories of mechanical and structural flaws.

<br>
![xwingdrag]({{ site.baseurl }}/assets/images/shyam/tridentissues.png)
{: .text-center}
<br>
<p align="center" class="small-text">
<em>Figure 34: Trident Design Issues.</em>
</p>



## 1. Mechanical Control System Failure
The design used mid-body SG90 servos connected to the rear fins via long aluminum tie-rods. This linkage system introduced two critical, non-viable faults:
* **Hysteresis or "Control Slop"**: This is a fatal flaw for a precision control system. Hysteresis is the difference between the commanded position from the flight computer and the actual position of the fin. It was caused by the "stack-up" of three mechanical errors:
    * **Servo Inaccuracy**: The low-cost SG90 servos have poor precision.
    * **Linkage Play**: Minor gaps in the 3D-printed hinges.
    * **Tie-Rod Flex**: Even aluminum rods flex under the 25 m/s aerodynamic load. This "slop" means the control system is never certain of the fin's true position, leading to oscillation ("hunting") and an inability to make fine adjustments.
* **Aerodynamic Flutter**: A flexible, "springy" linkage (the tie-rod) connected to an aerodynamic surface (the fin) creates a classic recipe for flutter. At 25 m/s, the air pressure will deflect the fin, which flexes the rod. The rod springs back, overshooting the neutral position, where the air catches it again. This creates a high-frequency, self-exciting oscillation that would destroy the servo gears or the fin itself.

## 2. Structural Integrity Failure
The design made two trade-offs for serviceability and weight that rendered the airframe non-viable.
* Stress Concentration: To house the four servos, a large, rectangular cutout was made in the main fuselage. This "gaping hole" acts as a massive stress concentration point. Any impact force, especially on the nose, would be channeled directly to the sharp corners of this cutout, guaranteeing a structural fracture.
* Loss of Airframe Rigidity: The use of zip ties to save 7.5g was a critical error. An active projectile must be a **rigid body** for its control system to function.

The conclusion was absolute: a successful design required a **direct-drive system (servos at the fins) and a **rigid, structurally-sound body.**

</details>


---

## V7 HUNTER (Active Dart) 


<br>
![hunter Real]({{ '/assets/images/shyam/Hunterreal.jpg' | relative_url }})
{: .text-center}
<br>
<p align="center" class="small-text">
<em>Figure 35: 3D printed scale model of HUNTER</em>
</p>

<br>
![hunter CAD]({{ '/assets/images/shyam/huntercad.png' | relative_url }})
{: .text-center}
<br>
<p align="center" class="small-text">
<em>Figure 36: 3D design of HUNTER.</em>
</p>


Prototype 7, "Hunter," is the final, optimized design that **synthesizes all lessons from the previous six iterations**. It directly solves the critical failures of "Trident" (P6) by replacing the flawed linkage system with a **robust direct-drive servo system** at the tail.

Its key innovation is a structurally-integrated, **interlocking carbon fiber (CF) wing assembly**, which provides both aerodynamic stability and the primary structural rigidity for the entire airframe, eliminating the flex and fragility of P6. 

Combined with a multi-material body **(TPU64D/PEBA90A)** and a dedicated electronics chassis that guarantees stability, "Hunter" is our finalized, competition-ready projectile.

<div style="text-align: center;">
<iframe width="560" height="315" src="https://www.youtube.com/embed/eCNTes4OTjM?si=q-Hss8DO36VEV8yd" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
</div>

<p class="figure-caption" style="text-align:center;">
Media 02: Video of Hunter Split-Up (New Segmented Design)
</p>


---
<details markdown="1">
<summary style="font-size: 1.3rem; font-weight: 450; color: #0059b3;"><strong> In-Depth Analysis on HUNTER (Click to view)</strong></summary>


"Hunter" represents the culmination of our iterative design process. It implements the successful "rear-fin active control" philosophy from Prototype 6 while systematically engineering solutions for all of P6's identified mechanical and structural failures.
## 1. Solution to Mechanical Control Failures
"Trident's" (P6) greatest flaw was its mid-body servo linkage, which caused control hysteresis (slop) and was highly susceptible to aerodynamic flutter.
"Hunter" solves this by implementing a direct-drive system. The four servos are mounted at the extreme rear of the fuselage, with their output horns directly actuating the control surfaces. This design eliminates all mechanical linkages, resulting in a zero-slop, rigid, and highly responsive control system that is immune to linkage-based flutter.
## 2. Solution to Structural Integrity Failures
"Trident" was critically fragile due to two main flaws:
* Servo Cutouts: The "gaping hole" in the fuselage created a massive stress concentration point, guaranteeing fracture on impact.
* Zip-Tie Assembly: The flexible zip-ties used to replace traditional steel screws resulted in a non-rigid airframe with torsional flex, making it uncontrollable.
"Hunter" solves both problems. By moving the servos, the main body is now a structurally-sound cylindrical tube, which is vastly superior at distributing impact forces. Second, the zip-ties are replaced by a multi-functional interlocking CF wing system. These two "egg-crate" wing assemblies not only act as fins but also as the primary load-bearing supports for the modular fuselage, creating a stiff, robust monocoque-like structure.
## 3. Advanced Systems and Materials Integration
"Hunter" is the first prototype to achieve full, optimized systems integration:
* CG Management: A dedicated internal chassis holds all heavy electronics (Battery, Camera, PCB) at the extreme front of the projectile. This is a critical design feature that guarantees passive static stability by ensuring the Center of Gravity (CG) is placed well forward of the Center of Pressure (CP) (located at the rear fins).
* Multi-Material Optimization: "Hunter" moves beyond a single-material design for superior performance.
    * **Main Body: TPU64D** (High impact-absorption)
    * **Fuselage: PEBA90A** (Lightweight and rigid)
    * **Wings: Carbon Fiber** (Maximum stiffness)
* Control Surface Durability: The movable control surfaces are printed from TPU64D. This is a deliberate engineering trade-off. While a rigid material might offer a "crisper" response, the flexible TPU ensures the flaps will not shatter on impact, prioritizing durability in a competition environment.


</details>


---
<details markdown="1">
<summary style="font-size: 1.3rem; font-weight: 450; color: #0059b3;"><strong> Multi Material Innovation </strong></summary>

<br>
![multi]({{ site.baseurl }}/assets/images/shyam/multi.png)
{: .text-center}
<br>
<p align="center" class="small-text">
<em>Figure 37: Multi Material Innovative Projectile Design.</em>
</p>


Previous **single-material designs using TPU95A** were a compromise. While readily available, **TPU95A** lacks the rigidity for a precision airframe (causing body flex and control "slop") and is too brittle for high-velocity impacts.

Our **"Hunter" (P7) prototype** solves this with a multi-material "zonal" design, selecting advanced materials for specific functional advantages.

**Main Body & Control Surfaces (TPU64D)**: We chose this semi-rigid thermoplastic for its superior stiffness and exceptional toughness.

* The **rigidity** is essential to prevent airframe flex and control surface "blow back" under aerodynamic load, ensuring a responsive projectile.

* The **toughness** ensures the body and flaps can survive repeated, high-velocity impacts without shattering.

**Internal Fuselage / Electronics Chassis (PEBA90A)**: The primary driver for this internal component was **minimizing weight.**

* This non-impact structure is "parasitic mass," and TPU is unnecessarily dense.

* PEBA90A was chosen for its **exceptionally low density** (compared to TPU), which saved critical grams and allowed us to remain under the 350g mass limit.

This multi-material approach (TPU64D for stiffness, PEBA90A for low weight) resulted in a final projectile that is more rigid, durable, and lighter than a single-material design.

</details>