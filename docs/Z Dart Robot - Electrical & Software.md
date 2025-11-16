---
title: 7 Electrical & Software - Dart Robot
parent: Dart Robot
grand_parent: Dart System
nav_order: 4
layout: default
permalink: /dart-system/dart-robot/elec-software
---
## 7.0 Introduction
<br>
**Objective:** Identify, design, and implement the electrical and software requirements for the dart robot based on user needs.

**Deliverables:**
1. Define the dart robot functionalities and propose the methodology 
→ [See Section 7.1](#func-breakdown)
2. Design a functional prototype based on the proposed methodology
 → [See Section 7.2](#func-prototype)
3. Implement the designed prototype
→ [See Section 7.3](#implement-prototype)

---

## 7.1 Function Definition and User Interaction {#func-breakdown}

### 7.1.1 Background

Dart robot users control the dart robot to shoot dart projectiles. The **user journey** maps out how the user operates the dart robot and the points where each function is required.


### 7.1.2 User Journey

<br>

<div style="text-align: center;">
<iframe width="560" height="315" src="https://www.youtube-nocookie.com/embed/FLCwOA101J8?si=7lEBV499AkXXTPhk" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
</div>

<p class="figure-caption" style="text-align:center;">
Figure 7.1: Video of User Journey
</p>


From the User Journey, the functions below were identified.

<table>
  <tr>
    <th>Function</th>
    <th>Mechanism / Description</th>
  </tr>

  <tr>
    <td>Pitch and Yaw Motor Control</td>
    <td>
      Control pitch and yaw motor movement based on user input, with an interface that accepts commands from either the remote controller or the touchscreen.
    </td>
  </tr>

  <tr>
    <td>Remote Controller Input Handling</td>
    <td>
      Read and process pitch, yaw, and launch commands from the remote controller.
    </td>
  </tr>

  <tr>
    <td>Touchscreen Input Interface</td>
    <td>
      A GUI captures pitch, yaw, and tension inputs from the touchscreen.
    </td>
  </tr>

  <tr>
    <td>System Status and Feedback Display</td>
    <td>
      Update the touchscreen with motor status, angles, tension, and lock state, and show errors for motor or sensor faults (Not shown in Figure 7.1 but useful).
    </td>
  </tr>

  <tr>
    <td>Elastic Band Launching Mechanism Control</td>
    <td>
      The launching motor drives and positions the solenoid lock, secures the elastic band, regulates tension using load-cell feedback until the set force is reached, and releases the lock when the launch command is triggered.
    </td>
  </tr>
</table>

<div style="text-align:center;">
Table 7.1: User functions and their corresponding electrical and software mechanisms
</div>


However, four constraints were identified when attempting to implement the five key functions:

1. Uncertainty regarding how the different functions would integrate.  
2. High implementation cost due to limited available funds.  
3. Requirement to hit a moving target with high precision and accuracy.  
4. Motor selection from the mechanical team scheduled only between November and December.  

### 7.1.3 Design Methodology

Given the above constraints, the **critical function prototyping methodology** was selected to prioritise integration validation. This approach tackle each constraint by:

1. Verifying core subsystem interfaces early, reducing integration uncertainty and confirming module interoperability before full-system development.  
2. Restricting the prototype to essential functions, preventing premature expenditure on modules that may later face integration issues.  
3. Isolating the key components that must operate reliably easily and allows early tuning of those elements, while providing data such as the required band force and corresponding band length needed to hit the target.
4. Allowing electrical–mechanical interface work to proceed using interim or simplified actuation while the final motor selection is pending.  

---

## 7.2 Design of functional prototype {#func-prototype}

### 7.2.1 Design Specifications

The design specifications were drafted based on the critical function prototype methodology, focusing only on the parameters needed to validate subsystem integration and core functionality.

<br>


| **Features** | **Specification** | **Reasoning** |
|--------------|-------------------|---------------|
| Maximum Power Supply Voltage (V) | 30 | Follows the building specification requirements from RMOC [1]. |
| Maximum Total Power Capacity (Wh) | 300 | Follows the building specification requirements from RMOC [1]. |
| Cost of individual items to be bought | < $20 | Provides a financial buffer for crucial industrial mechanical and electrical components in the next prototype. |
| Electronic Lock Holding force |  > 2 × 23 kgF | Safety factor of 2 and based on 23 kgF derived from the launching mechanical subsystem. |
| Load cell exictation voltage | 5 V | Common excitation voltage for load cells. **Swappable** once the rated load cell is determined, requiring minimal code/electrical changes. |
| Touch Screen Electrical Interface| Easy interface with RoboMaster’s Development Board A [2] |  Simplifies wiring with existing hardware. |


<div style="text-align:center;">
Table 7.2: Design Specifications for the Critical Function Prototype
</div>

<br>


### 7.2.2 Stakeholder Provided Items
As the project is used for NUS Calibur Robotics, several items were provided by the stakeholder.
<br>

| **Items** | **Reasons** |
|----------|-------------|
|<img src="{{ '/assets/images/barry/TB48S.png' | relative_url }}" width="140"> <br> **Battery (TB48S)** | ✅ Maximum Power Supply Voltage (24V) < 30 V <br> ✅ Maximum Total Power Capacity (129.96Wh) < 300Wh <br><br> *Design specifications met* |
| <img src="{{ '/assets/images/barry/dev_a.png' | relative_url }}" width="140"> <br> **RoboMaster Development Board A** | 1. Stakeholder ease of operation <br> 2. Easy electrical interface with other stakeholder provided items  <br> 3. Widely available in the stakeholder’s workshop |
| <img src="{{ '/assets/images/barry/RoboMaster_Remote_Controller.png' | relative_url }}" width="140"> <br> **RoboMaster Remote Controller** | 1. Stakeholder ease of operation <br> 2. Well-documented, with a provided receiver driver <br> 3. Widely available in the stakeholder’s workshop |
| <img src="{{ '/assets/images/barry/M2006_motor.png' | relative_url }}" width="140"> <br> **M2006 Motor** | 1. Lightweight, easy to bring around for prototyping <br> 3. Well-documented, with a provided motor driver  <br> 2. Widely available in the stakeholder’s workshop |

<div style="text-align:center;">
Table 7.3:  Stakeholder Provided Items and justification for using them
</div>

<br>

### 7.2.3 Electronic Lock & Load Cell

| **Items** | **Reasons** |
|----------|-------------|
| <img src="{{ '/assets/images/barry/Electronic_solenoid_lock.png' | relative_url }}" width="140"> <br> **Electronic Solenoid Lock** | ✅ Rated 75 kgF — Holding force > 2 × 23 kgF (meets safety factor) <br><br> ✅ (Cost: \$10.94) < \$20 <br><br> **Design specifications met** |
| <img src="{{ '/assets/images/barry/Beam_type_load_cell_with_ADC.png' | relative_url }}" width="140"> <br> **Load Cell with Analog-to-Digital Converter** | ✅ Rated 5V excitation voltage — meets 5V system excitation requirement <br><br> ✅ (Cost: \$2.59) < \$20 <br><br> **Design specifications met**|

<div style="text-align:center;">
Table 7.4: Bought items & Reasons
</div>


### 7.2.4 Touch Screen GUI

As per the user journey, a touch screen GUI is required to enter values for pitch, yaw, and force.  
The implementation of this function requires three primary components:

1. A touchscreen display  
2. A compatible driver  
3. A graphical user interface (GUI) library  
<br>

##### Touchscreen display / Compatible driver

Balancing between the RAM limitations of the STM32F4 microcontroller in the RoboMaster’s Development Board A, the type of touch sensor, and the screen size, a list of touchscreens was narrowed down in the table below.

| Specification | TFT LCD 2.4" resistive | **TFT LCD 3.5" resistive** | Hosyond 3.5" IPS capacitive |
|--------------|-------------------------|--------------------------|------------------------------|
| Resolution   | 240×320                 | 480×320                  | 480×320                     |
| Touch        | Resistive               | Resistive                | Capacitive                  |
| Touch IC     | XPT2046                 | XPT2046                  | FT6336U                     |
| Display IC   | ILI9341                 | ILI9488                  | ST7796U                     |
| Communication| SPI                     | SPI                      | SPI                         |
| Cost         | $13.71                  | $17.85                   | $45.23                      |

<div style="text-align:center;">
Table 7.5: Touch Screen Comparison
</div>

When comparing screen sizes, the 3.5-inch display is preferred over the 2.4-inch because it provides more buffer for future improvements, with the tradeoff being higher RAM usage.  
Capacitive touch screens are also far more expensive than resistive ones, and since capacitive touch is not a hard requirement, the **3.5-inch LCD TFT with resistive touch** was selected.

<br>

#### GUI library

When looking for a graphics user interface, a few libraries came up. Based on library usability, three options were shortlisted for comparison in the table below.

| Criteria                          | <img src="{{ '/assets/images/barry/LVGL_logo.png' | relative_url }}" width="140"> | <img src="{{ '/assets/images/barry/Touch_GFX_logo.png' | relative_url }}" width="140"> | <img src="{{ '/assets/images/barry/QT_logo.png' | relative_url }}" width="140"> |
| --------------------------------- | --------- | --------- | --------- |
| Ease of integration               | 3         | 3         | 1         |
| RAM / flash efficiency            | 3         | 2         | 1         |
| Graphics performance              | 2         | 3         | 3         |
| Customisation                     | 3         | 2         | 3         |
| Community support                 | 3         | 2         | 3         |
| Tools                             | 2         | 3         | 3         |
| Licensing                         | 3         | 3         | 1         |
| **Sum**                           | **19**    | 18    | 15    |

<div style="text-align:center;">
Table 7.6: GUI Library Comparison
</div>

Based on the selection criteria, it was a close decision between LVGL and TouchGFX. LVGL being very flexible and TouchGFX is optimise for stm32 chips hardware. With the given STM32F4,  the advantage that touchGFX has in stm32 chips could not be used. This is coupled with the desire of more flexibility in visual design that LVGL offers. Hence,  **LVGL** was chosen as the GUI library.

<br>

### 7.2.4 Module Testing and Code Development

After selecting the required components, each part was implemented and tested with dedicated code to verify correct operation before system integration. You can view the [testing videos](https://www.youtube.com/playlist?list=PLNnqZhGC3D1AK7iFgLpn61OiXpmc4Bh3g) here.



<div style="width:600px; margin:auto;">
<table style="width:60%; font-size:0.8rem;">
  <tr><th>Module</th><th>Tests / Code Written</th></tr>
  <tr><td>Touchscreen</td><td>Touch read, refresh check</td></tr>
  <tr><td>Load cell</td><td>ADC read, calibration</td></tr>
  <tr><td>Solenoid lock</td><td>Unlocking, holding control</td></tr>
  <tr><td>Motor control</td><td>Direction, joystick</td></tr>
  <tr><td>Remote controller</td><td>Signal read, switch mapping</td></tr>
</table>
</div>

<div style="text-align:center;">
Table 7.7: Module testing and code development table 
</div>

---

## 7.3 Implementation of designed functional prototype {#implement-prototype}

### 7.3.1 First Critical Function Prototype
![Critical_function_prototype]({{ "/assets/images/barry/Critical_function_prototype.jpg" | relative_url }})
<p class="figure-caption" style="text-align:center;">
Figure 7.2: Critical Function Prototype
</p>

<div style="text-align:center;">
<iframe width="560" height="315" src="https://www.youtube.com/embed/74U8P2-_PPw?si=B43NDyxnVTX-11rO" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
</div>

<p class="figure-caption" style="text-align:center;">
Figure 7.3: Critical Function Prototype Video
</p>

### 7.3.2 Current electrical architecture

From the prototype, the dart robot electrical diagram is shown below

<div style="text-align:center;">
  <img src="{{ '/assets/images/barry/Segmented_power_diagram.png' | relative_url }}" style="width:60%;" />
  <p class="figure-caption">Figure 7.4: Segmented Power Diagram of Dart Robot</p>
</div>

<div style="text-align:center;">
  <img src="{{ '/assets/images/barry/Dart_robot_signal.png' | relative_url }}" style="width:60%;" />
  <p class="figure-caption">Figure 7.5: Dart Robot Signal Overview</p>
</div>


### 7.3.3 Current software architecture

Likewise, the software architecture is shown below

<div style="text-align:center;">
  <img src="{{ '/assets/images/barry/Data_flow_diagram.jpg' | relative_url }}" style="width:60%;" />
  <p class="figure-caption">Figure 7.6: Dart Robot Data Flow Diagram</p>
</div>

The data flow diagram (DFD) illustrates how data moves within the Dart Robot system. At Level 0, user commands from the remote control, force data from the load cell, and lock status from the solenoid are processed by the Dart Robot, which outputs motor speed, motor status, and display data to the resistive touchscreen. 

| **Task / ISR** | **Functionalities** | **Input / Output** |
|----------------|----------------------|---------------------|
| Remote Control ISR | Captures controller readings from the RoboMaster Remote Controller | **Input:** User commands<br>**Output:** Unlock commands |
| Force Sensor Task | Reads load-cell data and computes force values for launch control. | **Input:** Force data<br>**Output:** Force data |
| Solenoid Control Task | Controls lock engagement and release states. | **Input:** Lock status <br>**Output:** Lock status, Lock control |
| Motor Control Task | Set the motor speed of the motor based on the desired speed | **Input:**  Desired Motor Speed, Motor Status <br>**Output:** Motor Speed|
| Launching Control Task | Spins the motor to pull the elastic band until the target force is reached | **Input:** Force data, Lock status <br>**Output:** Desired Motor Speed |
| Resistive Touchscreen Task | Updates graphical interface with system state and measurements. | **Input:** Force data,  Lock status<br>**Output:** Display Data |

<div style="text-align:center;">
Table 7.7: Summary of Dart Robot software tasks and their data inputs and utputs
</div>

### 7.3.4 Limitations
The current prototype tested the integration of the different components and demonstrated the expected overall behaviour. However, the strict electrical and software requirements for the final dart robot have not yet been met, as these depend on mechanical specifications that will be available only from November to end December. Once those specifications are provided, software modifications should be straightforward. The motor control task will require only a driver change, and the load cell replacement will require only recalibration.

The control method shown in the prototype video uses simple bang-bang logic. A full elastic-band model must be developed before selecting an appropriate controller. In addition, the feeding subsystem controls have not yet been designed.

The prototype also revealed a limitation in the current GUI implementation. The selected GUI library updates correctly during the initial write but does not refresh on subsequent writes, preventing real-time parameter display. The touch-sensing component also shows inaccurate position detection and requires calibration, which will be carried out during the winter holidays. Both issues must be addressed before integrating the GUI into the full control system.
<br>
### 7.3.5 Future work
1. Tension control for the elastic band
2. Source a load cell that meets technical requirements once determined
3. Source and Implement motors (i.e yaw, pitch, feeding, launching) 
4. GUI library and touch functions need to be fixed
