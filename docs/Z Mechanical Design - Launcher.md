---
title: "6&nbsp;Mechanical&nbsp;Design&nbsp;–&nbsp;Launcher"
parent: Dart Robot
grand_parent: Dart System
nav_order: 3     # after “Dart Robot - Mechanical Design” and “Dart Robot - Electrical & Software”
layout: default
permalink: /dart-system/dart-robot/launcher
has_toc: true
---

## Launcher Design

## 1. Foreground
The launcher is the sub-system responsible for imparting the required initial velocity to the projectile. The design of this sub-system is governed by two primary sets of constraints. First are the size and safety restrictions imposed by the RMOC rules. Second is the critical operational requirement from NUS Calibur Robotics for a propulsion system that delivers an exceptionally **consistent and reliable** launch velocity, as this is the most significant variable for ensuring target accuracy.

---

## 2. Special Requests
The launcher is the sub-system responsible for imparting the required initial velocity to the projectile. The design of this sub-system is governed by two primary sets of constraints. First are the size and safety restrictions imposed by the RMOC rules. Second is the critical operational requirement from NUS Calibur Robotics for a propulsion system that delivers an exceptionally **consistent and reliable** launch velocity, as this is the most significant variable for ensuring target accuracy.

There is a critical operational constraint dictating the launcher's required cycle time. During a match, the DART robot's holding station gate opens for a **15-second launch window**, and this opportunity occurs only **twice** per match.
Per the NUS Calibur Robotics request, the system must be capable of loading, aiming, and launching **two projectiles** within this brief window. This imposes a maximum "gate-to-gate" cycle time of **7.5 seconds per projectile**, demanding a system capable of rapid reloading and firing.


<div style="text-align: center;">
<iframe width="560" height="315" src="https://www.youtube.com/embed/V6OpgE-5DA0?si=c8CVtrMg53wsoJX3" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
</div>
<p class="figure-caption" style="text-align:center;">
Media 01: Video of Dart Robot Holding Area Gate Open 
</p>

---

## 3. Prior Analysis 

Before developing our innovative solution, a "prior art" analysis was conducted on the four primary propulsion methods prevalent in the 2024 RMUC competition season.

* **Flywheel**

https://www.youtube.com/shorts/sFb6AtxOMYo

<br>
![flywheel]({{ '/assets/images/shyam/flywheel.png' | relative_url }})
{: .text-center}
<br>
<p align="center" class="small-text">
<em>Figure 1: NUS 2024 UREX Dart team flywheel launcher</em>
</p>


This method uses two pairs of counter-rotating flywheels to grip and accelerate the projectile. Our own NUS 2024 predecessor team used this design, and our analysis identified a significant control challenge: achieving and maintaining **perfect RPM synchronization** across four independent motors is non-trivial. Any minor RPM mismatch introduces an asymmetric force, imparting an uncontrolled spin or yaw to the projectile and destroying accuracy.

* **Tension Spring & Compression Spring**

<br>
![tension]({{ '/assets/images/shyam/newspring.png' | relative_url }})
{: .text-center}
<br>
<p align="center" class="small-text">
<em>Figure 2, 3, 4: Tension and Compression Spring based launchers</em>
</p>


These systems store energy in large mechanical springs.
**Compression springs** *push* the projectile from its base. This is dynamically unstable, as any off-axis force can cause the projectile to "fishtail" (yaw) during its travel on the rail. **Tension springs** *pull* the projectile from a sled, which is aerodynamically superior as it guarantees a stable, straight-line exit. However, both spring types were rejected due to **serviceability and mechanical flaws**. The large, centralized springs are difficult to access for replacement. Furthermore, their hook-and-ring attachment points introduce mechanical tolerances (slop) that can lead to inconsistent energy transfer.


* **Compressed Air** (Pneumatic) Systems 

<br>
![air]({{ '/assets/images/shyam/air.png' | relative_url }})
{: .text-center}
<br>
<p align="center" class="small-text">
<em>Figure 4: Theoretical Compressed air based launcher</em>
</p>

Pneumatic systems offer high power but were deemed non-viable for two reasons. Firstly they require a near-perfect cylindrical projectile to form an effective pressure seal, which severely restricts the aerodynamic design freedom of our projectile. Secondly, the primary issue is transportation. Bringing high-pressure compressed air tanks to an overseas competition in China from Singapore presents significant customs and air-travel-regulation challenges.

---

## 4. Our Team's Plan: A Novel Elastic Launcher Design 
Based on the shortcomings of existing systems, our team developed an innovative propulsion system based on elastic energy.

### 4.1 Launching Method Justification
The primary energy-storage medium chosen was **latex elastic bands** over mechanical springs. This decision was based on the superior **energy density** (energy-to-mass ratio) of latex. A metal spring capable of storing the required launch energy would be prohibitively heavy and bulky. Latex bands provide this same high-energy capacity at a fraction of the mass, enabling a lightweight and transportable launcher.

<br>
![air]({{ '/assets/images/shyam/latex.png' | relative_url }})
{: .text-center}
<br>
<p align="center" class="small-text">
<em>Figure 5: Latex Tubes and Compound Bow launchers by Competitor teams, and Latex Exercise Bands by OUR Team</em>
</p>


This approach was validated as the 2025 competition saw the emergence of latex-tube and compound-bow launchers. Our design innovated on this by selecting commercially available, high-tensile **latex exercise bands**. This material is not only high-force but is also globally available and poses no transportation or customs issues.

### 4.2. Mechanism of Action

<div style="text-align: center;">
<iframe width="560" height="315" src="https://www.youtube.com/embed/13IwM2ret3M?si=JKrDa7dKXEccvA6-" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
</div>
<p class="figure-caption" style="text-align:center;">
Media 02: Our Own Launcher Prototype Design and Functionality 
</p>

Our launcher uses a motor-driven leadscrew to pull a carriage containing a **load cell** and a **solenoid lock**. This carriage latches onto the projectile sled and draws it back, stretching the elastic bands.

Our core innovation is a **force-based control loop**, which solves the inconsistencies of **traditional distance-based systems**. In distance-based launchers, band fatigue (from use, heat, or light) causes a fixed draw length to store progressively less energy, destroying velocity consistency.

Our system, however, pulls to a pre-set **force target** (e.g., 50 kgF). The motor draws the sled back until the load cell measures this exact force, at which point the solenoid releases. This **self-correcting system** automatically compensates for band degradation by pulling further, guaranteeing **identical stored potential energy** and **consistent launch velocity** on every shot. The system also includes a safety feature that warns the operator when the draw distance exceeds a preset limit, indicating the band needs replacement.


<details markdown="1">
<summary style="font-size: 1.5rem; font-weight: 450; color: #0059b3;"><strong> 📈5. Mathematical Calculations📈 (Click here) </strong></summary>


## 5.1. Material Justification


To develop a predictive mathematical model for the launcher, it was necessary to **experimentally characterize** the force-displacement relationship of the selected Decathlon exercise band. The manufacturer does not provide engineering specifications such as a spring constant (k) or Young's Modulus, making empirical testing necessary.
This characterization was achieved by securing the band to a test rig equipped with a luggage scale. The band was drawn to fixed displacement intervals (draw lengths), and the corresponding elastic tension force was recorded. The resulting force-displacement data (Table [X]) forms the basis for all subsequent energy and trajectory calculations.


<br>
![experiment]({{ site.baseurl }}/assets/images/shyam/exper.png)
{: .text-center}
<br>
<p align="center" class="small-text">
</p>

<br>
![tabledart]({{ site.baseurl }}/assets/images/shyam/tabledart.png)
{: .text-center}
<br>
<p align="center" class="small-text">
<em>Figure 6: Experimental Data Table for Latex Exercise Band.</em>
</p>

## 5.2. Mathematical Analysis

***Projectile and Environmental Variables***
To model the projectile's trajectory, several key parameters are defined. The projectile mass (m) is set to the competition limit of 350g (0.35 kg). The aerodynamic reference area($$S. A_{projectile}$$)  is defined as the frontal cross-sectional area. This value is calculated in Solidworks by summing the areas of all surfaces projected normal to the flight vector.
The drag coefficient (CD) is a complex, non-linear function of shape, Reynolds Number (Re), and Angle of Attack (p). Given our low operational velocity of 25m/s (Mach < 0.3), the flow is treated as incompressible. Based on the streamlined shape of our "Hunter" prototype, a constant CD value of 0.35 is assumed as a standard approximation for this analysis. Both m, ($$S. A_{projectile}$$) , and CD are treated as key input variables in the subsequent mathematical model.

<br>
![surfacearea]({{ site.baseurl }}/assets/images/shyam/surfacearea.png)
{: .text-center}
<br>
<p align="center" class="small-text">
<em>Figure 7: Frontal Surface Area of HUNTER Projectile.</em>
</p>

$$m_{projectile} = 350g = 0.35kg$$

$$S.A_{projectile} = 2459.47mm^2 = 0.00245947m^2$$

Drag Coefficient $$C_D = 0.35$$

Density of Air $$\rho = 1.225kg/m^3$$

Gravitational Force $$g = 9.81m/s^2$$

Dynamic Viscosity $$\mu = 1.81 \times 10^{-5} Pa$$

Reynolds Number $$Re = \frac{\rho v L}{\mu}$$

whereby $$v$$ = velocity of projectile

$$L$$ = Length of projectile


For this trajectory analysis, the 6-DOF problem is simplified to a 2-DOF point-mass model based on two key assumptions:

Lift Coefficient (CL) is assumed to be Zero. Our "Glider" (P2) prototype proved that uncontrolled lift is catastrophic for accuracy. 

The "Hunter" (P7) is an axisymmetric, non-lifting body by design. 

We are therefore modeling a purely ballistic trajectory. Rotational Dynamics are Omitted. We are treating the projectile as a point mass, not a rigid body. This simplification is justified because the projectile's proven passive stability (CG forward of CP) keeps it aligned with the flight vector. Therefore, moments of inertia (Ixx, Iyy, Izz) are not relevant to this primary trajectory calculation.

<br>
![launchermath]({{ site.baseurl }}/assets/images/shyam/launchermath.png)
{: .text-center}
<br>
<p align="center" class="small-text">
<em>Figure 8: Simplified Illustration of Projectile Motion.</em>
</p>

***Launcher Mathematics***

**IF** the latex band's spring constant k was known, 

***Work Energy Theorem***


$$W_{Band} = KE_{Projectile} = \frac{1}{2}mv_0^2 \text{, whereby } v_0 = \text{exit velocity of projectile}$$

$$W_{Band} = PE_{Band} = \int_{0}^{L} F(x)dx \text{, whereby } L = \text{draw length}$$

$$F_{Release} = kL \quad \therefore k = \frac{F_{Release}}{L}$$

$$PE_{Band} = \int_{0}^{L} F(x)dx = \int_{0}^{L} kxdx = \int_{0}^{L} \frac{F_{Release}}{L}xdx = (\frac{F_{Release}}{L})(\frac{x^2}{2})\Big|_{0}^{L} = \frac{1}{2}F_{Release}L$$

$$W_{Band} = PE_{Band} = \frac{1}{2}F_{Release}L$$

$$W_{Band} = KE_{Projectile} = \frac{1}{2}mv_0^2$$

$$\therefore \frac{1}{2}F_{Release}L = \frac{1}{2}mv_0^2$$

$$\therefore v_0 = \sqrt{\frac{F_{Release}L}{m_{projectile}}}$$

Adhering to RMOC rules on the launcher design, *Angle of attack 45°*, *Draw Length 0.75m*. Adhering to the Decathalon Band safety warning before tear, $$F_{Release}$$60KgF = 588.6N.

---

***Elastic Potential Energy Analysis***

As mentioned under section [5.1 Figure 6], we had to experimentally characterise the Decathalon exercise band. We will use an Approximate Integral Numerically using **Trapezoidal Rule**. The P.E.(L) function is the potential energy function for the elastic band with respect to the drawlength. 

$$PE_{Band}(L) = \sum_{i=1}^{n} \frac{1}{2} [F(x_i) + F(x_{i-1})] * [x_i - x_{i-1}] \text{, whereby } L = x_n \text{ is the total draw length}$$

This provides a high fidelity P.E.(L) function from the experimental data from Figure 6.

with efficiency $$\eta = 0.8$$ and a safety factor of $$1.2$$,

$$K.E. = \eta * P.E.(L)$$

$$\frac{1}{2}mv_0^2 = \eta * P.E.(L)$$

$$\therefore v_0(L) = \sqrt{\frac{2 * \eta * P.E.(L)}{m_{projectile}}}$$

This is the exit velocity as a function of the draw length $$L$$.

---

***Projectile flight dynamics with quadratic drag***

https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/flight-equations-with-drag/

$$F_D = \frac{1}{2} \rho v^2 C_D A \text{, whereby}$$

$$\rho$$: density of air = $$1.225kg/m^3$$

$$v$$: instantaneous speed of projectile = $$\sqrt{v_x^2 + v_y^2}$$ , $$v_x$$ and $$v_y$$ being the x and y vectors of the velocity of projectile

$$C_D$$: assumed drag coefficient = $$0.35$$

$$A$$: projectile's frontal area

---

***Equation of Motion***

Modelling the projectile's trajectory by applying Newton's second law formula $$F = m.a$$.
Resolve forces in x & y components

Define drag constant as $$k = \frac{1}{2} \rho C_D A$$

$$F_D = kv^2 = k(v_x^2 + v_y^2)$$

Forces in x direction accounting for drag(x direction)

$$drag \text{ } force \text{ } F_{D(x)} = -F_D \cos\theta = -F_D (\frac{v_x}{v})$$
$$ma_x = -kv^2(\frac{v_x}{v}) = -kvv_x = -k\sqrt{v_x^2 + v_y^2} \cdot v_x$$

$$\therefore \text{ (1) --------- } a_x = \frac{dv_x}{dt} = -\frac{k}{m}v_x\sqrt{v_x^2 + v_y^2}$$

Forces in y direction accounting for drag(x direction) and gravity

$$ma_y = -mg - F_D \sin\theta = -mg - F_D (\frac{v_y}{v}) = -mg - kv^2(\frac{v_y}{v}) = -mg - kvv_y$$

$$\therefore \text{ (2) --------- } a_y = \frac{dv_y}{dt} = -g - \frac{k}{m}v_y\sqrt{v_x^2 + v_y^2}$$


These two equations (1) and (2) form a system of coupled, non-linear 2nd-order Ordinary
Differential Equations (ODE). This system does not have simple, close-form analytical solutions.

Therefore to find the trajectory, we must solve the ODEs numerically. We will employ a time-marching numerical method. We will simulate the projectile's state $$(x, y, v_x, v_y)$$ at discrete time steps $$\Delta t$$. A more common **ROBUST method is the 4th-order Runge-Kutta (RK4) Algorithm**. A simpler Euler method can be used as well, but it is less stable. This numeric model forms the core of our Direct Fire Control System, which will predict the point(s) of impact based on our initial conditions.

***Solution finding algorithm***

There are two variable parameters, ANGLE OF ATTACK $$\alpha$$ and DRAW LENGTH $$L$$. ANGLE OF ATTACK $$\alpha$$ determines the angle our Dart' PITCH needs to go to, and the DRAW LENGTH $$L$$ is necessary to decide how long the launcher needs to be. The goal is to find a pair of ($$\alpha, L$$) that results in a HIT.
In our established coordinate system Figure (8), our target position is (25.591, 1.1165)m. We will do a grid search:


$$\frac{dx}{dt} = v_x \quad \frac{dy}{dt} = v_y \quad \frac{dv_x}{dt} = a_x = -\frac{k}{m}v_x v \quad \frac{dv_y}{dt} = a_y = -g - \frac{k}{m}v_y v$$


Our algorithm features a Get StateDerivatives(S) function, which gets state vector S and returns
derivative vector $$[\frac{dx}{dt}, \frac{dy}{dt}, \frac{dv_x}{dt}, \frac{dv_y}{dt}]$$. Then we use linear interpolation:


$$y_{HIT!} = S_{previous}.y + (S.y - S_{previous}.y) \cdot \frac{X_{Target} - S_{previous}.x}{S.x - S_{previous}.x}$$



<details markdown="1">
<summary><strong> View Python Code: Trajectory Solver (solver.py)</strong></summary>

<br>
![multi]({{ site.baseurl }}/assets/images/shyam/first.png)
{: .text-center}
<br>
<p align="center" class="small-text">
</p>
<br>
![multi]({{ site.baseurl }}/assets/images/shyam/second.png)
{: .text-center}
<br>
<p align="center" class="small-text">
</p>
<br>
![multi]({{ site.baseurl }}/assets/images/shyam/last.png)
{: .text-center}
<br>
<p align="center" class="small-text">
</p>

</details>

<br>
![multi]({{ site.baseurl }}/assets/images/shyam/resultp.png)
{: .text-center}
<br>
<p align="center" class="small-text">
<em>Launch code output.</em>
</p>


From the code result, we would require 20kgF force input and a ***launch angle of 43.00°.***

---

***Launcher Release Mechanism Mathematics***

This was done to determine the time taken to pull back a projectile and release it. Since the hard limit was to launch a singular projectile in 7.5seconds, and the feeder would also take time to LOAD one projectile into the launcher, we decided that the drawback of the projectile has to be **within 2.5 seconds.** 

The torque required to drive a leadscrew against a linear force:

$$T = \frac{F \cdot P}{2\pi\eta}$$

$$T$$ = Torque (N·m)
$$F$$ = Resistive Force (N)
$$P$$ = Pitch (m/rev)
$$\eta$$ = Leadscrew efficiency = 0.85 for ballscrew

<br>
The mechanical power required to overcome the resistive force:

$$P_{out} = F \cdot v$$

$$F$$ = Resistive Force (N)
$$v$$ = Linear velocity (m/s)

<br>
The electrical power the motor must provide, accounting for efficiency:

$$P_{in} = \frac{P_{out}}{\eta} = T \cdot \omega$$

$$\omega$$ = Angular velocity (rad/s)

<br>
Max Force $$F_{max} = 23kgF \times 9.81 m/s^2 = 225.63N$$

Drawback length (L) = 0.750m

Time to achieve = 2.5s

Pitch (P) = 100mm/rev = 0.1m/rev

Linear speed of drawback $$v_{linear} = L/t = 0.750m / 2.5s = 0.3m/s$$

Revolutions per second (rev/s) : $$v_{linear} / P = (0.3m/s) / (0.1m/rev) = 3.0rev/s$$

Revolutions per minute (RPM of motor) : $$3.0rev/s \times 60s/min = \mathbf{180 RPM}$$

To find PEAK TORQUE $$T_{max} = \frac{F_{max} \cdot P}{2 \cdot \pi \cdot \eta} = \mathbf{4.23 Nm}$$

</details>

## 6. Prototype and Results 

The novel "force-based" elastic launcher concept required a low-cost, physical prototype to validate its feasibility. Our mathematical model confirmed that the final design would require a high-torque motor and a high-pitch leadscrew, both of which are high-cost components.

*https://www.igus.sg/product/drylin_SD_DST_LS_R_ES?artnr=DST-LS-18X100-R-ES*
*https://www.foxtechrobotics.com/damiao-j8009p-2ec-mit-driven-brushless-servo-joint-motor*

To preserve the project's limited budget ($3200) for the final, flight-ready hardware, this initial prototype was fabricated entirely from on-hand scrap materials and spare workshop parts, bringing the prototype's material cost to **$0.**

<br>
![launchvalid]({{ '/assets/images/shyam/launchvalid.png' | relative_url }})
{: .text-center}
<br>
<p align="center" class="small-text">
<em>Figure 9: Launcher Testing at EA</em>
</p>


The **objective** was to *validate the core "force-based" release concept*, observe the projectile's release characteristics to **ensure a "clean" separation from the sled**, and identify any immediate, unforeseen mechanical failures or abnormal behaviors in the elastic system.


<div style="text-align: center;">
<iframe width="560" height="315" src="https://www.youtube.com/embed/K77tQGB8lvU?si=WDydWgSiRGhgS4xm" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
</div>
<p class="figure-caption" style="text-align:center;">
Media 03: Video of our own Prototype Launcher Testing 
</p>


<br>
![launchvalid2]({{ '/assets/images/shyam/launchvalid2.png' | relative_url }})
{: .text-center}
<br>
<p align="center" class="small-text">
<em>Figure 10: Launch Results</em>
</p>


Initial tests of the prototype were highly successful. A **23 kgF (225.6 N)** launch force propelled the "Crossblade" projectile to a distance of **20 meters**.Two critical observations were made, firstly the projectile's trajectory was stable and linear, with **no visible roll**. Secondly the projectile maintained a consistent nose-forward "dipping" arc throughout its flight. The projectile had no issues when releasing from the 3D printed “sled”. These results confirmed that the elastic band concept is a promising and viable solution for this project.


<br>
![launchvalid3]({{ '/assets/images/shyam/launchvalid3.png' | relative_url }})
{: .text-center}
<br>
<p align="center" class="small-text">
<em>Figure 11: Launcher "Sled", handle and aluminium carrier damage</em>
</p>


The tests revealed a critical design oversight, which is the lack of a deceleration system for the launch sled.

After releasing the projectile, the sled continued at full velocity and impacted the launcher's front plate. This high-g dead stop impact resulted in the 3D-printed sled to shear and completely be destroyed.

Secondly the scrap-aluminum carrier, which held the sled, was visibly deforming and buckling under the high tensile force of the 23 kgF.

To solve these issues, two corrective actions were immediately implemented for the next iteration. The *aluminum carrier* will be replaced with a **high-strength steel plate** to withstand the band's peak tensile load, and a **spring-based damping system** will be designed and integrated into the launcher's front plate to safely absorb the sled's kinetic energy and bring it to a controlled stop.
