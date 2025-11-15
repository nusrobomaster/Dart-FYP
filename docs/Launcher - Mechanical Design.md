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


## 2. Special Requests
The launcher is the sub-system responsible for imparting the required initial velocity to the projectile. The design of this sub-system is governed by two primary sets of constraints. First are the size and safety restrictions imposed by the RMOC rules. Second is the critical operational requirement from NUS Calibur Robotics for a propulsion system that delivers an exceptionally **consistent and reliable** launch velocity, as this is the most significant variable for ensuring target accuracy.

There is a critical operational constraint dictating the launcher's required cycle time. During a match, the DART robot's holding station gate opens for a **15-second launch window**, and this opportunity occurs only **twice** per match.
Per the NUS Calibur Robotics request, the system must be capable of loading, aiming, and launching **two projectiles** within this brief window. This imposes a maximum "gate-to-gate" cycle time of **7.5 seconds per projectile**, demanding a system capable of rapid reloading and firing.


Embed 15 second video here of dartgate open


## 3. Prior Analysis 

Before developing our innovative solution, a "prior art" analysis was conducted on the four primary propulsion methods prevalent in the 2024 RMUC competition season.

* Flywheel
Embed video
photo
This method uses two pairs of counter-rotating flywheels to grip and accelerate the projectile. Our own NUS 2024 predecessor team used this design, and our analysis identified a significant control challenge: achieving and maintaining **perfect RPM synchronization** across four independent motors is non-trivial. Any minor RPM mismatch introduces an asymmetric force, imparting an uncontrolled spin or yaw to the projectile and destroying accuracy.

* Tension Spring & Compression Spring

These systems store energy in large mechanical springs.
**Compression springs** *push* the projectile from its base. This is dynamically unstable, as any off-axis force can cause the projectile to "fishtail" (yaw) during its travel on the rail. **Tension springs** *pull* the projectile from a sled, which is aerodynamically superior as it guarantees a stable, straight-line exit. However, both spring types were rejected due to **serviceability and mechanical flaws**. The large, centralized springs are difficult to access for replacement. Furthermore, their hook-and-ring attachment points introduce mechanical tolerances (slop) that can lead to inconsistent energy transfer.

* Compressed Air (Pneumatic) Systems 
images
Pneumatic systems offer high power but were deemed non-viable for two reasons. Firstly they require a near-perfect cylindrical projectile to form an effective pressure seal, which severely restricts the aerodynamic design freedom of our projectile. Secondly, the primary issue is transportation. Bringing high-pressure compressed air tanks to an overseas competition in China from Singapore presents significant customs and air-travel-regulation challenges.

## 4. Our Team's Plan: A Novel Elastic Launcher Design 
Based on the shortcomings of existing systems, our team developed an innovative propulsion system based on elastic energy.

# 4.1 Launching Method Justification
The primary energy-storage medium chosen was **latex elastic bands** over mechanical springs. This decision was based on the superior **energy density** (energy-to-mass ratio) of latex. A metal spring capable of storing the required launch energy would be prohibitively heavy and bulky. Latex bands provide this same high-energy capacity at a fraction of the mass, enabling a lightweight and transportable launcher.

images 

This approach was validated as the 2025 competition saw the emergence of latex-tube and compound-bow launchers. Our design innovated on this by selecting commercially available, high-tensile **latex exercise bands**. This material is not only high-force but is also globally available and poses no transportation or customs issues.

# 4.2. Mechanism of Action
Insert Video

Our launcher is an electro-mechanical system using a motor-driven leadscrew to actuate a carriage. This carriage integrates two key components: a high-precision **load cell** (force sensor) and a **solenoid lock** (release trigger).
The automated release cycle begins when the solenoid latches onto the projectile sled. The motor then reverses, drawing the sled backward and stretching the elastic bands.
Our core innovation is a **force-based control loop**, which solves the energy inconsistency of traditional **distance-based** systems. In distance-based launchers, latex band fatigue (from duty cycles, heat or direct light exposure) means a fixed pull-back length results in progressively less stored energy, destroying velocity consistency.
Our system pulls to a pre-set **force target** (e.g., 50 kgF). The motor draws the sled back until the load cell measures this exact force, at which point the solenoid releases. This **self-correcting system** automatically compensates for band degradation by pulling it further to reach the target, guaranteeing **identical stored potential energy** and **consistent launch velocity** for every shot. Additionally, the system can warn the operator when the draw distance exceeds a preset limit, indicating the band needs replacement.


<details markdown="1">
<summary style="font-size: 1.5rem; font-weight: 450;"><strong> 5. Mathematical Calculations </strong></summary>


# 5.1. Material Justification


To develop a predictive mathematical model for the launcher, it was necessary to **experimentally characterize** the force-displacement relationship of the selected Decathlon exercise band. The manufacturer does not provide engineering specifications such as a spring constant (k) or Young's Modulus, making empirical testing necessary.
This characterization was achieved by securing the band to a test rig equipped with a luggage scale. The band was drawn to fixed displacement intervals (draw lengths), and the corresponding elastic tension force was recorded. The resulting force-displacement data (Table [X]) forms the basis for all subsequent energy and trajectory calculations.


table

# 5.2. Mathematical Analysis

***Projectile and Environmental Variables***
To model the projectile's trajectory, several key parameters are defined. The projectile mass (m) is set to the competition limit of 350g (0.35 kg). The aerodynamic reference area($S. A_{projectile}$)  is defined as the frontal cross-sectional area. This value is calculated in Solidworks by summing the areas of all surfaces projected normal to the flight vector.
The drag coefficient (CD) is a complex, non-linear function of shape, Reynolds Number (Re), and Angle of Attack (p). Given our low operational velocity of 25m/s (Mach < 0.3), the flow is treated as incompressible. Based on the streamlined shape of our "Hunter" prototype, a constant CD value of 0.35 is assumed as a standard approximation for this analysis. Both m, $S. A_{projectile}$ , and CD are treated as key input variables in the subsequent mathematical model.

2 images

$m_{projectile} = 350g = 0.35kg$

$S.A_{projectile} = 2459.47mm^2 = 0.00245947m^2$

Drag Coefficient $C_D = 0.35$

Density of Air $\rho = 1.225kg/m^3$

Gravitational Force $g = 9.81m/s^2$

Dynamic Viscosity $\mu = 1.81 \times 10^{-5} Pa$

Reynolds Number $Re = \frac{\rho v L}{\mu}$

whereby $v$ = velocity of projectile

$L$ = Length of projectile


For this trajectory analysis, the 6-DOF problem is simplified to a 2-DOF point-mass model based on two key assumptions:
Lift Coefficient (CL) is assumed to be Zero. Our "Glider" (P2) prototype proved that uncontrolled lift is catastrophic for accuracy. The "Hunter" (P7) is an axisymmetric, non-lifting body by design. We are therefore modeling a purely ballistic trajectory. Rotational Dynamics are Omitted. We are treating the projectile as a point mass, not a rigid body. This simplification is justified because the projectile's proven passive stability (CG forward of CP) keeps it aligned with the flight vector. Therefore, moments of inertia (Ixx, Iyy, Izz) are not relevant to this primary trajectory calculation.

image

***Launcher Mathematics***
**IF** the latex band's spring constant k was known, 

Work Energy Theorem (only use this if elastic band’s elastic coefficient k is known)


$$W_{Band} = KE_{Projectile} = \frac{1}{2}mv_0^2 \text{, whereby } v_0 = \text{exit velocity of projectile}$$

$$W_{Band} = PE_{Band} = \int_{0}^{L} F(x)dx \text{, whereby } L = \text{draw length}$$

$$F_{Release} = kL \quad \therefore k = \frac{F_{Release}}{L}$$

$$PE_{Band} = \int_{0}^{L} F(x)dx = \int_{0}^{L} kxdx = \int_{0}^{L} \frac{F_{Release}}{L}xdx = (\frac{F_{Release}}{L})(\frac{x^2}{2})\Big|_{0}^{L} = \frac{1}{2}F_{Release}L$$

$$W_{Band} = PE_{Band} = \frac{1}{2}F_{Release}L$$

$$W_{Band} = KE_{Projectile} = \frac{1}{2}mv_0^2$$

$$\therefore \frac{1}{2}F_{Release}L = \frac{1}{2}mv_0^2$$

$$\therefore v_0 = \sqrt{\frac{F_{Release}L}{m_{projectile}}}$$

Adhering to RMOC rules on the launcher design, Angle of attack 45° , Draw Length 0.75m. Adhering to the Decathalon Band safety warning before tear, FRelease60KgF = 588.6N.

***Elastic Potential Energy Analysis***
As mentioned under section [ ], we had to experimentally characterise the Decathalon exercise band. We will use an Approximate Integral Numerically using Trapezoidal Rule. The P.E.(L) function is the potential energy function for the elastic band with respect to the drawlength. 

$$PE_{Band}(L) = \sum_{i=1}^{n} \frac{1}{2} [F(x_i) + F(x_{i-1})] * [x_i - x_{i-1}] \text{, whereby } L = x_n \text{ is the total draw length}$$

This provides a high fidelity P.E.(L) function from the experimental data from section [ ].

with efficiency $\eta = 0.8$ and a safety factor of $1.2$,

$$K.E. = \eta * P.E.(L)$$

$$\frac{1}{2}mv_0^2 = \eta * P.E.(L)$$

$$\therefore v_0(L) = \sqrt{\frac{2 * \eta * P.E.(L)}{m_{projectile}}}$$

This is the exit velocity as a function of the draw length $L$.

**Projectile flight dynamics with quadratic drag**
Add reference links from NASA

$$F_D = \frac{1}{2} \rho v^2 C_D A \text{, whereby}$$

$\rho$: density of air = $1.225kg/m^3$

$v$: instantaneous speed of projectile = $\sqrt{v_x^2 + v_y^2}$ , $v_x$ and $v_y$ being the x and y vectors of the velocity of projectile

$C_D$: assumed drag coefficient = $0.35$

$A$: projectile's frontal area


***Equation of Motion***
Modelling the projectile's trajectory by applying Newton's second law formula $F = ma$.
Resolve forces in x & y components

Define drag constant as $k = \frac{1}{2} \rho C_D A$
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

Therefore to find the trajectory, we must solve the ODEs numerically. We will employ a time-marching numerical method. We will simulate the projectile's state $(x, y, v_x, v_y)$ at discrete time steps $\Delta t$. A more common ROBUST method is the 4th-order Runge-Kutta (RK4) Algorithm. A simpler Euler method can be used as well, but it is less stable. This numeric model forms the core of our Direct Fire Control System, which will predict the point(s) of impact based on our initial conditions.

***Solution finding algorithm***

There are two variable parameters, ANGLE OF ATTACK $\alpha$ and DRAW LENGTH $L$. ANGLE OF ATTACK $\alpha$ determines the angle our Dart' PITCH needs to go to, and the DRAW LENGTH $L$ is necessary to decide how long the launcher needs to be. The goal is to find a pair of ($\alpha, L$) that results in a HIT.
In our established coordinate system Table [], our target position is (25.591, 1.1165)m. We will do a grid search:

$$\frac{dx}{dt} = v_x \quad \frac{dy}{dt} = v_y \quad \frac{dv_x}{dt} = a_x = -\frac{k}{m}v_x v \quad \frac{dv_y}{dt} = a_y = -g - \frac{k}{m}v_y v$$

Our algorithm features a Get StateDerivatives(S) function, which gets state vector S and returns
derivative vector $[\frac{dx}{dt}, \frac{dy}{dt}, \frac{dv_x}{dt}, \frac{dv_y}{dt}]$. Then we use linear interpolation:

$$y_{HIT!} = S_{previous}.y + (S.y - S_{previous}.y) \cdot \frac{X_{Target} - S_{previous}.x}{S.x - S_{previous}.x}$$



<details markdown="1">
<summary><strong>► View Python Code: Trajectory Solver (solver.py)</strong></summary>

import numpy as np
from scipy.integrate import solve_ivp
from scipy.interpolate import interp1d

# --- 1. SET CONSTANTS  ---

# Projectile & Environment
m = 0.35              # mass (kg)
S_A = 0.00245947      # Frontal area (m^2)
C_D = 0.35            # Drag coefficient
rho = 1.225           # Air density (kg/m^3)
g = 9.81              # Gravity (m/s^2)

# Launcher
eta = 0.8             # Launcher efficiency
g_force = 9.81        # To convert kgF to Newtons

# Target
x_target = 25.591     # Target x-position (m)
y_target = 1.1165     # Target y-position (m)

# Drag constant k 
k = 0.5 * rho * C_D * S_A

# --- 2. LAUNCHER ENERGY MODEL (From data table) ---

# Experimental data: Distance (m) and Force (Newtons)
dist_data = np.array([0.00, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 
                        0.35, 0.40, 0.45, 0.50, 0.55, 0.60])
force_data = np.array([0.00, 1.39, 2.78, 4.17, 6.00, 8.50, 10.66, 
                        12.30, 13.93, 15.57, 17.21, 18.85, 20.00]) * g_force

# Create a high-fidelity interpolation function for the band
force_interpolator = interp1d(dist_data, force_data, kind='linear')

def get_v0(L):
    """
    Calculates the exit velocity (v0) for a given draw length (L)
    by integrating the experimental force-displacement data.
    """
    x_integral = np.linspace(0, L, num=100)
    f_integral = force_interpolator(x_integral)
    Ep = np.trapz(f_integral, x_integral)
    v0 = np.sqrt(2 * eta * Ep / m)
    return v0

# --- 3. FLIGHT DYNAMICS MODEL (The ODEs) ---

def getStateDerivative(t, S):
    """
    Calculates the derivatives of the state vector S = [x, y, vx, vy]
    """
    x, y, vx, vy = S
    v = np.sqrt(vx**2 + vy**2)
    ax = -(k / m) * v * vx
    ay = -g - (k / m) * v * vy
    return [vx, vy, ax, ay]

# --- 4. SIMULATION AND SOLVER ---

def simulate_shot(alpha, L):
    """
    Simulates a single shot and returns (y_hit, t_hit) at the target.
    """
    v0 = get_v0(L)
    vx0 = v0 * np.cos(np.radians(alpha))
    vy0 = v0 * np.sin(np.radians(alpha))
    S0 = [0, 0, vx0, vy0]
    
    def stop_at_target(t, S):
        return S[0] - x_target
    stop_at_target.terminal = True
    stop_at_target.direction = 1
    
    t_span = [0, 5]
    sol = solve_ivp(getStateDerivative, t_span, S0, 
                      events=stop_at_target, 
                      dense_output=True)
    
    # FIX for DeprecationWarning
    if sol.t_events[0].size == 0:
        return (-999, -1)  # Return error values
        
    t_hit = sol.t_events[0][0]
    S_hit = sol.sol(t_hit)
    y_hit = S_hit[1]
    
    return (y_hit, t_hit)

# --- 5. GRID SEARCH (Find the optimal parameters) ---

print("Running grid search to find optimal launch parameters...")
print(f"Target: ({x_target} m, {y_target} m)")

alpha_range = np.linspace(10, 45, 36)  # 36 steps (1 degree)
L_range = np.linspace(0.2, 0.6, 41)    # 41 steps (1 cm)

best_error = float('inf')
best_alpha = 0
best_L = 0
best_y_hit = 0
best_t_hit = 0

for alpha in alpha_range:
    for L in L_range:
        y_hit, t_hit = simulate_shot(alpha, L)
        
        if y_hit == -999:
            continue
            
        error = abs(y_hit - y_target)
        
        if error < best_error:
            best_error = error
            best_alpha = alpha
            best_L = L
            best_y_hit = y_hit
            best_t_hit = t_hit

# --- 6. PRINT FINAL RESULTS ---

# Calculate the final parameters based on the best solution
final_v0 = get_v0(best_L)
final_force_N = force_interpolator(best_L)
final_force_kgF = final_force_N / g_force

print("\n--- OPTIMAL SOLUTION FOUND ---")
print(f"Best Angle (α):         {best_alpha:.2f} degrees")
print(f"Best Draw Length (L):     {best_L:.3f} meters")
print("\n--- PREDICTED LAUNCH VALUES ---")
print(f"Required Band Force:    {final_force_kgF:.2f} kgF ({final_force_N:.2f} N)")
print(f"Resulting Exit Velocity:  {final_v0:.2f} m/s")
print(f"Predicted Flight Time:  {best_t_hit:.3f} seconds")
print("\n--- PREDICTED IMPACT ---")
print(f"Predicted Hit Height:   {best_y_hit:.4f} meters")
print(f"Target Height:            {y_target:.4f} meters")
print(f"Error (miss distance):  {best_error:.4f} meters")

</details>

insert image of code output

From the code result, we would require 20kgF force input and a launch angle of 43.00°.

***Launcher Release Mechanism Mathematics***
This was done to determine the time taken to pull back a projectile and release it. Since the hard limit was to launch a singular projectile in 7.5seconds, and the feeder would also take time to LOAD one projectile into the launcher, we decided that the drawback of the projectile has to be **within 2.5 seconds.** 

The torque required to drive a leadscrew against a linear force:

$$T = \frac{F \cdot P}{2\pi\eta}$$

$T$ = Torque (N·m)
$F$ = Resistive Force (N)
$P$ = Pitch (m/rev)
$\eta$ = Leadscrew efficiency = 0.85 for ballscrew

<br>
The mechanical power required to overcome the resistive force:

$$P_{out} = F \cdot v$$

$F$ = Resistive Force (N)
$v$ = Linear velocity (m/s)

<br>
The electrical power the motor must provide, accounting for efficiency:

$$P_{in} = \frac{P_{out}}{\eta} = T \cdot \omega$$

$\omega$ = Angular velocity (rad/s)

<br>
Max Force $F_{max} = 23kgF \times 9.81 m/s^2 = 225.63N$

Drawback length (L) = 0.750m

Time to achieve = 2.5s

Pitch (P) = 100mm/rev = 0.1m/rev

Linear speed of drawback $v_{linear} = L/t = 0.750m / 2.5s = 0.3m/s$

Revolutions per second (rev/s) : $v_{linear} / P = (0.3m/s) / (0.1m/rev) = 3.0rev/s$

Revolutions per minute (RPM of motor) : $3.0rev/s \times 60s/min = \mathbf{180 RPM}$

To find PEAK TORQUE $T_{max} = \frac{F_{max} \cdot P}{2 \cdot \pi \cdot \eta} = \mathbf{4.23 Nm}$

</details>

## 6. Prototype and Results 

The novel "force-based" elastic launcher concept required a low-cost, physical prototype to validate its feasibility. Our mathematical model confirmed that the final design would require a high-torque motor and a high-pitch leadscrew, both of which are high-cost components.
https://www.igus.sg/product/drylin_SD_DST_LS_R_ES?artnr=DST-LS-18X100-R-ES
https://www.foxtechrobotics.com/damiao-j8009p-2ec-mit-driven-brushless-servo-joint-motor
To preserve the project's limited budget ($3200) for the final, flight-ready hardware, this initial prototype was fabricated entirely from on-hand scrap materials and spare workshop parts, bringing the prototype's material cost to $0.

pics

The objective was to validate the core "force-based" release concept, observe the projectile's release characteristics to ensure a "clean" separation from the sled, and identify any immediate, unforeseen mechanical failures or abnormal behaviors in the elastic system.

Embedded Video of launcher

pics

Initial tests of the prototype were highly successful. A 23 kgF (225.6 N) launch force propelled the "Crossblade" projectile to a distance of 20 meters.Two critical observations were made, firstly the projectile's trajectory was stable and linear, with no visible roll. Secondly the projectile maintained a consistent nose-forward "dipping" arc throughout its flight. The projectile had no issues when releasing from the 3D printed “sled”. These results confirmed that the elastic band concept is a promising and viable solution for this project.


pics

The tests revealed a critical design oversight, which is the lack of a deceleration system for the launch sled.
After releasing the projectile, the sled continued at full velocity and impacted the launcher's front plate. This high-g dead stop impact resulted in the 3D-printed sled to shear and completely be destroyed .Secondly the scrap-aluminum carrier, which held the sled, was visibly deforming and buckling under the high tensile force of the 23 kgF.
To solve these issues, two corrective actions were immediately implemented for the next iteration. The aluminum carrier will be replaced with a high-strength steel plate to withstand the band's peak tensile load, and a spring-based damping system will be designed and integrated into the launcher's front plate to safely absorb the sled's kinetic energy and bring it to a controlled stop.
