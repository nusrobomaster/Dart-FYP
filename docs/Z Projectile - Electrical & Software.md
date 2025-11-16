---
title: 3 Electrical & Software
parent: Projectile
grand_parent: Dart System
nav_order: 2
layout: default
permalink: /dart-system/projectile/elec-software
has_toc: true
---

## 3 Projectile – Electrical

### 3.1 Introduction
This section focuses on the electrical and software systems inside the projectile. It covers the design of the custom power and signal distribution PCB, integration of the OpenMV camera, and the development of the code that converts camera-based target deviations into servo movements.

### 3.2 Design Objectives
- Deliver stable and reliable power to all onboard components.  
- Process image data fast enough to support in-flight corrections.  
- Drive four/six PWM-controlled fins using hardware PWM.  
- Maintain stable operation during launch and flight.  
- Support fast maintenance and debugging without major disassembly.
<br>

### 3.3 Design Specifications
<div style="width:600px; margin:auto;">
<br>
<table>
  <thead>
    <tr>
      <th style="text-align:center;">Feature</th>
      <th style="text-align:center;">Specification</th>
      <th style="text-align:center;">Reasoning</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td>Maximum Battery Voltage</td>
      <td>8.4V</td>
      <td>Competition Requirement</td>
    </tr>
    <tr>
      <td>Maximum Battery capacity</td>
      <td>4Wh</td>
      <td>Competition Requirement</td>
    </tr>
    <tr>
      <td>Battery capacity &amp; C-rating</td>
      <td>Must supply at least 8A continuous</td>
      <td>Supports all components under peak load with margin</td>
    </tr>
    <tr>
      <td>Voltage rails</td>
      <td>Supplies 3.3V / 5V / 7.4V</td>
      <td>Different components require different voltages for best performance</td>
    </tr>
    <tr>
      <td>Plug-and-play wiring</td>
      <td>All connectors and ports are labelled and fit inside the projectile</td>
      <td>Simplifies maintenance and replacement of damaged parts</td>
    </tr>
    <tr>
      <td>PCB form factor</td>
      <td>Custom PCB must fit fully inside the projectile</td>
      <td>Limited internal space requires a compact board</td>
    </tr>
    <tr>
      <td>PCB mounting</td>
      <td>PCB includes at least 2 mounting holes</td>
      <td>Reduces vibration and lowers the risk of electrical failure</td>
    </tr>
    <tr>
      <td>Onboard image processing</td>
      <td>Supports filtering and color tracking</td>
      <td>Enables the projectile to detect and track the target</td>
    </tr>
    <tr>
      <td>Servo control capability</td>
      <td>Must output at least six hardware PWM channels</td>
      <td>Provides smooth and reliable fin actuation</td>
    </tr>
    <tr>
      <td>Camera-to-servo latency</td>
      <td>Latency &lt; 50 ms</td>
      <td>Ensures servo response is fast enough for in-flight corrections</td>
    </tr>
  </tbody>
</table>
</div>
<p align="center" class="small-text"><em>Table 3-1: Design Specification Table</em></p>

### 3.4 Component Selection
#### 3.4.1 Dart Trigger
<br>
<p align="center">
  <img src="{{ '/assets/images/vj/Projectile/Dart_Trigger_Off_&_On_state.png' | relative_url }}" width="600">
</p>
{: .text-center}
<p align="center" class="small-text"><em>Figure 3-1: Dart Trigger Off & On State</em></p>

The Dart Trigger is an official RMOC competition module. It activates and emits a red light when the projectile experiences more than 5g of acceleration. This trigger signal is used by the launcher and projectile firmware to confirm launch events.

#### 3.4.2 CAM Boards

<br>
<p align="center">
  <img src="{{ '/assets/images/vj/Projectile/Board_Selection_Table.png' | relative_url }}" width="600">
</p>
<p align="center" class="small-text"><em>Figure: Board Selection Table</em></p>

{: .text-center}
<p align="center" class="small-text"><em>Table 3-2: Board Selection Table</em></p>

OpenMV boards are selected because they let you view camera footage directly from the SD card by connecting through USB. This avoids flashing different scripts or running a webserver, which would consume compute resources. It also speeds up debugging and testing, which is important during competition when time and manpower are limited.

The **OpenMV RT1062** is chosen over the N6 even though it has lower compute performance. The RT1062 supports up to six hardware PWM outputs, which gives room for future projectile designs that may require up to six fins.

*(See Appendix XX for the Design Considerations for Board Selection.)*

#### 3.4.3 Servo

<br>
<p align="center">
  <img src="{{ '/assets/images/vj/Projectile/Servo_Selection_Table.png' | relative_url }}" width="600">
</p>
{: .text-center}
<p align="center" class="small-text"><em>Table 3-3: Servo Selection Table</em></p>

**Blue Arrow** is chosen because it is the lightest, smallest, and fastest option. It also meets the required torque while staying within a reasonable price range.
(See Appendix XX for torque calculations). 

After testing if a higher torque servo is required, the KST and BlueArrow have similar servos at higher torque but with a slightly bigger form factor and reduced speed.

#### 3.4.4 Battery
<br>
<p align="center">
  <img src="{{ '/assets/images/vj/Projectile/Lipo_battery_used_in_Projectile.png' | relative_url }}" width="300">
</p>
{: .text-center}
<p align="center" class="small-text"><em>Figure 3-2 Lipo battery used in Projectile</em></p>

<br>
<p align="center">
  <img src="{{ '/assets/images/vj/Projectile/Battery_Specifications_Reasons_for_Battery_Selection.png' | relative_url }}" width="600">
</p>
{: .text-center}
<p align="center" class="small-text"><em>Table 3-4 Battery Specifications [Reasons for Battery Selection]</em></p>

The battery can safely support 7 launches. (see below for calculation).


##### 3.4.4.1 Calculations for Number of Launches the battery can safely Support
<br>
<p align="center">
  <img src="{{ '/assets/images/vj/Projectile/Energy_Usage_Breakdown_for Dart_Launcher_Components.png' | relative_url }}" width="600">
</p>
{: .text-center}
<p align="center" class="small-text"><em>Table 3-5 Energy Usage Breakdown for Dart Components</em></p>

<br>
Assumptions made:
1. Board is put to sleep 10 seconds after hitting the same target
2. User turns off the projectile within 1 minute of the end of the 7 minute round
<br>

Total energy used per launch

$$
E_{\text{launch}} = 74 + 50 + 840 = 964\ \text{W}
$$

---

Energy stored (usable 20%–100%)

$$
E_{\text{stored}} = (\text{Capacity} \times 0.8) \times 3600
$$

$$
= (3.33 \times 0.8) \times 3600
= 2.664\ \text{Wh} \times 3600
= 9590.4\ \text{W}
$$

---

Number of launches supported

$$
N = \frac{9590.4}{964} \div 1.4
$$

$$
= \frac{9.9485}{1.4}
= 7.1061
\approx 7\ \text{launches}
$$

---

##### 3.4.4.2 Calculated Battery Charging Time

$$
\text{Charge Time} = \left(\frac{450}{800}\right)\times 60
= 0.5625 \times 60
= 33.75\ \text{minutes}
$$

Actual tested charging time: **45–60 minutes**.

### 3.5 Custom PCB

<br>
<p align="center">
  <img src="{{ '/assets/images/vj/Projectile/Top_and_Bottom_View_of Custom_PCB.png' | relative_url }}" width="600">
</p>
{: .text-center}
<p align="center" class="small-text"><em>Table 3-6 Top and Bottom View of Custom PCB</em></p>

The PCB uses JST-PH 2.0 connectors. Users do not need to check individual pin connections before plugging in unlike jumper wires, making it faster and easier to debug and swap components.

<br>
<p align="center">
  <img src="{{ '/assets/images/vj/Projectile/Side_View_of_OpenMV_Board _and_Custom_PCB.png' | relative_url }}" width="600">
</p>
{: .text-center}
<p align="center" class="small-text"><em>Figure 3-3 Side View of OpenMV Board and Custom PCB</em></p>

The Custom PCB would be pressed on top of the OpenMV board. The PCB helps with the wire management as 7 Power, 6 GND and 4 Signal connections need to be made in a small space. 
It also has bulk and decoupling capacitors for each servo rail to reduce noise which results in smooth servo motion.

### 3.6 Hardware Overview

<br>
Dart consists of the following main electrical components.
<br>
<p align="center">
  <img src="{{ '/assets/images/vj/Projectile/Components_inside_dart.png' | relative_url }}" width="600">
</p>
{: .text-center}
<p align="center" class="small-text"><em>Figure 3-4 Components inside dart</em></p>

<br>
<p align="center">
  <img src="{{ '/assets/images/vj/Projectile/Wiring_Connections_of_Dart_Components.png' | relative_url }}" width="600">
</p>
{: .text-center}
<p align="center" class="small-text"><em>Figure 3-5 Wiring Connections of Dart Components</em></p>

<br>
<p align="center">
  <img src="{{ '/assets/images/vj/Projectile/Segmented_Signal_Diagram_of_Projectile.png' | relative_url }}" width="600">
</p>
{: .text-center}
<p align="center" class="small-text"><em>Figure 3-6 Segmented Signal Diagram of Projectile</em></p>
<br>
OpenMV Board processes the camera data from the camera and outputs PWM to control the 4 servos.


<br>
<p align="center">
  <img src="{{ '/assets/images/vj/Projectile/Segmented_Power_Diagram_of_Projectile.png' | relative_url }}" width="600">
</p>
{: .text-center}
<p align="center" class="small-text"><em>Figure 3-7 Segmented Power Diagram of Projectile</em></p>
<br>
The Battery provides 7.4V to the 4 servos and to the LM1085 voltage regulator which outputs 5V to the Dart Trigger and OpenMV Board.

### 3.7 Software Overview

For the Projectile to make active flight corrections it needs to do four things
1. Detect Target
2. Find X and Y deviation
3. Find Z deviation
4. Map X, Y, Z deviations to servo angles -> send to servo

All thrust comes from the launcher. The projectile does not generate any thrust of its own. Its only method to hit the target is to adjust its pitch and yaw servos so that the nose stays pointed toward the target, allowing it to steer toward the target during flight.

<br>
<p align="center">
  <img src="{{ '/assets/images/vj/Projectile/Y_and_Z_Deviation_Visualisation.png' | relative_url }}" width="600">
</p>
{: .text-center}
<p align="center" class="small-text"><em>Figure 3-8 Y & Z Deviation Visualisation</em></p>
<br>



<table style="width:100%;">
  <thead>
    <tr>
      <th style="text-align:center;">Deviation</th>
      <th style="text-align:center;">Description</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td style="text-align:left;">X (Left / Right)</td>
      <td style="text-align:left;">Horizontal deviation between projectile and target</td>
    </tr>
    <tr>
      <td style="text-align:left;">Y (Up / Down)</td>
      <td style="text-align:left;">Vertical deviation between projectile and target</td>
    </tr>
    <tr>
      <td style="text-align:left;">Z (Forward / Backward)</td>
      <td style="text-align:left;">Distance the projectile needs to travel forward to hit the target</td>
    </tr>
  </tbody>
</table>

<br>
<p align="center" class="small-text"><em>Table 3-7: Definitions of X , Y , Z deviations</em></p>
<br>

#### 3.7.1 Target Detection

<br>
<p align="center">
  <img src="{{ '/assets/images/vj/Projectile/Dart_Detection_Module.png' | relative_url }}" width="600">
</p>
{: .text-center}
<p align="center" class="small-text"><em>Figure 3-9 Close up of Dart Detection Module which is on the Targets (Outpost & Base)</em></p>
<br>

<br>
<p align="center">
  <img src="{{ '/assets/images/vj/Projectile/OpenMV_Threshold_Editor_Window.png' | relative_url }}" width="600">
</p>
{: .text-center}
<p align="center" class="small-text"><em>Figure 3-10 LAB Threshold Editor Window in OpenMV IDE</em></p>
<br>

A LAB colour space threshold filter is used to filter out everything except the green circular guiding light below the target. After filtering, only the target will be shown as white.

#### 3.7.2 Finding X & Y Deviation
<br>
<p align="center">
  <img src="{{ '/assets/images/vj/Projectile/Camera_View_from_Projectile.png' | relative_url }}" width="600">
</p>
{: .text-center}
<p align="center" class="small-text"><em>Figure 3-11 Camera View from Projectile + X & Y deviation visualization</em></p>
<br>

X & Y Deviation is the deviation from the centre of the camera feed (White +) and a set point slightly higher centre of the Target (Red +). They give the intermediate servo angles.
The more the X & Y deviation the more the yaw and pitch servo fins move.

The set point needs to be higher than the (Red +) as the projectile needs to hit the target plate (armour module) which is slightly higher than the green light that it detects and tracks.

#### 3.7.3 Finding Z Deviation
<br>
<p align="center">
  <img src="{{ '/assets/images/vj/Projectile/Major_and_Minor_Axis_Visualization.png' | relative_url }}" width="600">
</p>
{: .text-center}
<p align="center" class="small-text"><em>Figure 3-12 Major and Minor Axis Visualization</em></p>
<br>

The number of pixels along the major and minor axes of the target increases as the projectile gets closer.
Projectile–target distance is mapped from this pixel count, and a lookup table assigns a distance-dependent multiplier m. This m is applied to the intermediate servo angles to produce the final servo commands.
A smaller Z-axis deviation results in larger servo corrections.

### 3.8 Future Works

<br>
<div style="width:600px; margin:auto;">
<table>
  <thead>
    <tr>
      <th style="text-align:center;">Limitations</th>
      <th style="text-align:center;">Future Works</th>
      <th style="text-align:center;">Reasoning</th>
    </tr>
  </thead>

  <tbody>
    <!-- Row 1 -->
    <tr>
      <td>Current mapping between X, Y, Z deviations and servo angles is not fully tuned.</td>
      <td>Run controlled tests to refine the deviation-to-servo mapping.</td>
      <td>Better tuning improves in-flight corrections and increases hit consistency.</td>
    </tr>

    <!-- Row 2 -->
    <tr>
      <td>Target tracking is reliable only up to ~2 m due to minimum area threshold.</td>
      <td>Adjust camera resolution, exposure, and detection parameters to increase tracking distance.</td>
      <td>Beyond 2 m, the target occupies too few pixels and falls below detection threshold.</td>
    </tr>

    <!-- Row 3 -->
    <tr>
      <td>Projectile requires partial disassembly for power and USB access.</td>
      <td>Add a battery power switch and an external USB-C access port.</td>
      <td>Reduces debugging and maintenance time during field testing and competition.</td>
    </tr>
  </tbody>
</table>
<div>
<p align="center" class="small-text"><em>Table 3-8 Future Works Table</em></p>