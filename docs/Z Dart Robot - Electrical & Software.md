---
title: 7 Electrical & Software - Dart Robot
parent: Dart Robot
grand_parent: Dart System
nav_order: 4
layout: default
permalink: /dart-system/dart-robot/elec-software
---

# Objectives & Deliverables
<br>
**Objective:** Identify, design and implement electrical and software needs for the dart robot user

**Deliverables:**
1. Breakdown functionalities required in a dart robot and ideate for methodology required 
→ [See Section 7.1](#func-breakdown)
2. Design a functional prototype based on methodology
 → [See Section 7.2](#func-prototype)
3. Implement the designed functional prototype
→ [See Section 7.3](#implement-prototype)

---

## 7.1 Functionalities breakdown {#func-breakdown}

### 7.1.1 Background

Dart robot users need to control the dart robot so it can shoot dart projectiles stated in the introduction. To identify how and when the dart robot will be used, **user journey** gives you the perspective of the user.


### 7.1.2 User journey

<br>

<div style="text-align: center;">
<iframe width="560" height="315" src="https://www.youtube-nocookie.com/embed/FLCwOA101J8?si=7lEBV499AkXXTPhk" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
</div>

<p class="figure-caption" style="text-align:center;">
Figure 7.1: Video of User Journey
</p>

From the User Journey, 5 key functions were identified:

1. Tension feedback for the launcher  
2. Touch Screen Graphics User Interface for entering inputs  
3. Electrical lock to work with the launcher  
4. Remote control as an interface with the subsystems  
5. Motors to actuate different mechanisms  

However, four issues were identified when attempting to implement the five key functions:

1. Uncertainty regarding how the different functions would integrate.  
2. High implementation cost due to limited available funds.  
3. Requirement to hit a moving target with high precision and accuracy.  
4. Motor selection from the mechanical team scheduled only between November and December.  

### 7.1.3 Design Methodology

Given these constraints, the **critical function prototyping methodology** was selected to prioritise integration validation. This approach:

1. Verifies core subsystem interfaces early, reducing integration uncertainty and confirming module interoperability before full-system development.  
2. Restricts the prototype to essential functions, preventing premature expenditure on modules that may later face integration issues.  
3. Enables early evaluation of sensing and actuation paths, providing direct data on integration limits related to precision and accuracy.  
4. Allows electrical–mechanical interface work to proceed using interim or simplified actuation while the final motor selection is pending.  

---

## 7.2 Design of functional prototype {#func-prototype}

### 7.2.1 Design Specifications

<br>

| **Features** | **Specification** | **Reasoning** |
|--------------|-------------------|---------------|
| Maximum Power Supply Voltage (V) | 30 | Follows the building specification requirements from RMOC [1]. |
| Maximum Total Power Capacity (Wh) | 300 | Follows the building specification requirements from RMOC [1]. |
| Cost of individual items to be bought | < $20 | Provides a financial buffer for crucial industrial mechanical and electrical components in the next prototype. |
| Electronic Lock | Holding force > 2 × 23 kgF | Safety factor of 2 based on 23 kgF derived from the launching mechanical subsystem. |
| Load cell with ADC | 5 V excitation voltage | Common excitation voltage for load cells. **Swappable** once the rated load cell is determined, requiring minimal code/electrical changes. |
| Touch Screen | Easy interface with RoboMaster’s Dev Board A [1] | Stakeholder-provided development board for ease of use when handing over the dart robot. |

<div style="text-align:center;">
Table 7.1: Design Specifications
</div>

<br>


### 7.2.2 Choice of Items

#### 7.2.2.1 Stakeholder Provided Items
As the project is used for NUS Calibur Robotics, several items were provided by the stakeholder.
<br>

| **Items** | **Reasons** |
|----------|-------------|
|<img src="{{ '/assets/images/barry/TB48S.png' | relative_url }}" width="140"> <br> **Battery (TB48S)** | ✅ Maximum Power Supply Voltage (24V) < 30 V <br> ✅ Maximum Total Power Capacity (129.96Wh) < 300Wh <br><br> *Design specifications met* |
| <img src="{{ '/assets/images/barry/dev_a.png' | relative_url }}" width="140"> <br> **RoboMaster Development Board A** | Stakeholder ease of operation and widely available in the stakeholder’s workshop |
| <img src="{{ '/assets/images/barry/RoboMaster_Remote_Controller.png' | relative_url }}" width="140"> <br> **RoboMaster Remote Controller** | Stakeholder ease of operation and widely available in the stakeholder’s workshop |
| <img src="{{ '/assets/images/barry/M2006_motor.png' | relative_url }}" width="140"> <br> **M2006 Motor** | Widely available and lightweight, easy to bring around for prototyping |

<div style="text-align:center;">
Table 7.2:  Stakeholder Provided Items
</div>

<br>

#### 7.2.2.2 Choice of Bought items

##### 7.2.2.2.1 Lock & Load Cell

| **Items** | **Reasons** |
|----------|-------------|
| <img src="{{ '/assets/images/barry/Electronic_solenoid_lock.png' | relative_url }}" width="140"> <br> **Electronic Solenoid Lock** | ✅ Rated 75 kgF — Holding force > 2 × 23 kgF (meets safety factor) <br><br> ✅ (Cost: \$10.94) < \$20 <br><br> *Design specifications met* |
| <img src="{{ '/assets/images/barry/Beam_type_load_cell_with_ADC.png' | relative_url }}" width="140"> <br> **Load Cell with Analog-to-Digital Converter** | ✅ Rated 5V excitation voltage — meets 5V system excitation requirement <br><br> ✅ (Cost: \$2.59) < \$20 <br><br> *Design specifications met*|

<div style="text-align:center;">
Table 7.3: Bought items & Reasons
</div>


##### 7.2.2.2.2 Touch Screen GUI

As per the user journey, a touch screen GUI is required to enter values for pitch, yaw, and force.  
The implementation of this function requires three primary components:

1. A touchscreen display  
2. A compatible driver  
3. A graphical user interface (GUI) library  
<br>

###### 7.2.2.2.2.1 Touchscreen display / Compatible driver

Balancing between the RAM limitations of the STM32F4 microcontroller, the type of touch sensor, and the screen size, a list of touchscreens was narrowed down in the table below.

| Specification | TFT LCD 2.4" resistive | TFT LCD 3.5" resistive | Hosyond 3.5" IPS capacitive |
|--------------|-------------------------|--------------------------|------------------------------|
| Resolution   | 240×320                 | 480×320                  | 480×320                     |
| Touch        | Resistive               | Resistive                | Capacitive                  |
| Touch IC     | XPT2046                 | XPT2046                  | FT6336U                     |
| Display IC   | ILI9341                 | ILI9488                  | ST7796U                     |
| Communication| SPI                     | SPI                      | SPI                         |
| Cost         | $13.71                  | $17.85                   | $45.23                      |

<div style="text-align:center;">
Table 7.4: Touch Screen Comparison
</div>

When comparing screen sizes, the 3.5-inch display is preferred over the 2.4-inch because it provides more buffer for future improvements, with the tradeoff being higher RAM usage.  
Capacitive touch screens are also far more expensive than resistive ones, and since capacitive touch is not a hard requirement, the **3.5-inch LCD TFT with resistive touch** was selected.

<br>

###### 7.2.2.2.2.2 GUI library

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
Table 7.5: GUI Library Comparison
</div>

Based on the selection criteria, it was a close decision between LVGL and TouchGFX. LVGL being very flexible and TouchGFX optimise for stm32 chips hardware. With the given STM32F4,  the advantage that touchGFX has in stm32 chips could not be used. This is coupled with the desire of more flexibility in visual design that LVGL offers. Hence,  **LVGL** was chosen as the GUI library.

<br>

### 7.2.4 Module Testing and Code Development

After selecting the required components, each part was implemented separately and tested with dedicated code to verify correct operation before system integration.



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
Table 7.6: Touch Screen Comparison
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

<div style="text-align:center;">
  <img src="{{ '/assets/images/barry/Segmented_power_diagram.png' | relative_url }}" style="width:60%;" />
  <p class="figure-caption">Figure 7.4: Segmented Power Diagram of Dart Robot</p>
</div>

<div style="text-align:center;">
  <img src="{{ '/assets/images/barry/Dart_robot_signal.png' | relative_url }}" style="width:60%;" />
  <p class="figure-caption">Figure 7.5: Dart Robot Signal Overview</p>
</div>

---

### 7.3.3 Current software architecture

<div style="text-align:center;">
  <img src="{{ '/assets/images/barry/Data_flow_diagram.jpg' | relative_url }}" style="width:60%;" />
  <p class="figure-caption">Figure XX: Dart Robot Data Flow Diagram</p>
</div>

---

## Future work

- Tension control accuracy  
- Task prioritisation  
- Load cell upgrade  
- Motor upgrade  
- GUI + touch implementation remaining  
