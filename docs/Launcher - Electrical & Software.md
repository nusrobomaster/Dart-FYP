---
title: Launcher - Electrical & Software
parent: Launcher Subsystem
nav_order: 2
layout: default
permalink: /launcher/elec&software
---
<!-- nav_order is for DROPDOWN NAVIGATION sequence <--> 
# Electrical / Software

## User journey

Before going into the electrical and software, a user journey is required to understand the electrical and software needs.

A video of the user journey (narration over PowerPoint slides) is provided for reference.

<!-- ![Video of user journey](/assets/barry/user-journey-video-placeholder.png) -->
<p class="figure-caption" style="text-align:center;">
Figure XX: Video of User Journey
</p>

---

## Electrical and software requirements

From the user journey, five key functions were identified:

- Tension feedback for the launcher  
- Touch screen graphics user interface for entering inputs  
- Electrical lock to work with the launcher  
- Remote control as an interface with the subsystems  
- Motors to actuate different mechanisms  

However, there were three issues faced while trying to implement these five key functions:

- Uncertainty about integration of the different functions  
- High cost due to limited funds  
- Strict engineering requirement of hitting a moving target with precision and accuracy  

With the issues identified, the critical function prototyping methodology, focusing on validating the integration, was implemented.

---

## First critical function prototype

Following the chosen methodology, an initial prototype was built.

![Critical_function_prototype](/assets/images/barry/Critical_function_prototype.jpg)
<p class="figure-caption" style="text-align:center;">
Figure XX: Critical Function Prototype
</p>

A video of the prototype being used is also provided.

<!-- ![Video of current prototype](/assets/images/current-prototype-video-placeholder.png) -->
<p class="figure-caption" style="text-align:center;">
Figure XX: Video of Current Prototype
</p>

---

## Current electrical architecture

From the prototype, the dart robot electrical diagram is shown below.

<div style="text-align:center;">
  <img src="/assets/images/barry/Segmented_power_diagram.png" style="width:60%;" />
  <p class="figure-caption">Figure XX: Segmented Power Diagram of Dart Robot</p>
</div>


<div style="text-align:center;">
  <img src="/assets/images/barry/Dart_robot_signal.png" style="width:60%;" />
  <p class="figure-caption">Figure XX: Dart Robot Signal Overview</p>
</div>


---

## Current software architecture

Likewise, the software architecture is shown below.

<div style="text-align:center;">
  <img src="/assets/images/barry/Data_flow_diagram.jpg" style="width:60%;" />
  <p class="figure-caption">Figure XX: Dart Robot Data Flow Diagram</p>
</div>


The data flow diagram (DFD) illustrates how data moves within the Dart Robot system. At Level 0, user commands from the remote control, force data from the load cell, and lock status are processed by the Dart Robot, which outputs motor speed, motor status, and display data.

The Level 1 DFD expands this process into system tasks:

- Remote Control ISR  
- Force Sensor Task  
- Solenoid Control Task  
- Motor Control Task  
- Launching Control Task  
- Resistive Touchscreen Task  

---

## Tension feedback for the launcher

A cost-effective load cell with an ADC + PGA was used as a prototype. This allowed early development of the load cell interface code. When the final load cell is selected, only minor changes such as calibration will be needed.

<div style="text-align:center;">
  <img src="/assets/images/barry/Beam_type_load_cell_with_ADC.png" style="width:30%;" />
  <p class="figure-caption">Figure XX: Beam-type Load Cell with ADC</p>
</div>

---

## Touch screen GUI for entering inputs

As per the user journey, a touch screen GUI is required to enter values for pitch, yaw and force.
The implementation of this function requires three primary components:
1.	A touchscreen display

2.	A compatible driver

3.	A graphical user interface (GUI) library


### Touchscreen display and compatible driver
Balancing between the RAM limitations of the STM32F4 microcontroller, the type of touch sensor and the screen size, a list of touchscreens was narrowed in the table below.

| Specification                    | TFT LCD 2.4" resistive | TFT LCD 3.5" resistive | Hosyond 3.5" IPS capacitive |
| -------------------------------- | ------------------------ | ------------------------ | --------------------------- |
| Resolution                       | 240×320                  | 480×320                  | 480×320                     |
| Touch                            | Resistive                | Resistive                | Capacitive                  |
| Touch IC                         | XPT2046                  | XPT2046                  | FT6336U                     |
| Display IC                       | ILI9341                  | ILI9488                  | ST7796U                     |
| Communication                    | SPI                      | SPI                      | SPI                         |
| Cost                             | \$13.71                  | \$17.85                  | \$45.23                     |

<div style="text-align:center;">
  Table XX: Touch Screen Comparison
</div>
When comparing size of the screen, the 3.5 inch is better than the 2.4 inch as it gives more buffer for future improvements, with the tradeoff being more ram usage for the 3.5 inch. As for the type of touch, capacitive touch screens is much more expensive than the resistive ones. With capacitive touch not being a hard requirement, it was decided that it would be better to stick to the 3.5 inch LCD TFT with resistive touch.

### GUI library

When looking for a graphics user interface, a few libraries came up. Based on the library usability, I narrowed down to three libraries to use in the table below.


| Criteria                          | Library 1 | Library 2 | Library 3 |
| --------------------------------- | --------- | --------- | --------- |
| Ease of integration               | 3         | 3         | 1         |
| RAM / flash efficiency            | 3         | 2         | 1         |
| Graphics performance              | 2         | 3         | 3         |
| Customisation                     | 3         | 2         | 3         |
| Community support                 | 3         | 2         | 3         |
| Tools                             | 2         | 3         | 3         |
| Licensing                         | 3         | 3         | 1         |
| **Sum**                           | **19**    | **18**    | **15**    |

<div style="text-align:center;">
  Table XX: GUI Library Comparision
</div>
Based on the selection criteria, it was a close decision between LVGL and TouchGFX. LVGL being very flexible and TouchGFX optimise for stm32 chips hardware. With the given STM32F4,  the advantage that touchGFX has in stm32 chips could not be used. This is coupled with the desire of more flexibility in visual design that LVGL offers. Hence,  LVGL was chosen as the GUI library.

---

## Electrical lock to work with the launcher
As mentioned earlier in this report, the electrical solenoid lock satisfies the required mechanical specifications. For electrical integration with the TB48S power source, which operates at 24 V, a 24 V-to-12 V buck converter was used to supply power to the lock. The locking mechanism is released when an electrical relay connects power to the solenoid upon receiving a Transistor–Transistor Logic (TTL) high signal from a GPIO output. The lock state can be monitored by configuring the same GPIO pin as an input with an internal pull-up resistor. When the lock is engaged, the pin presents a high-impedance state, resulting in a logic low (0) reading.
<div style="text-align:center;">
  <img src="/assets/images/barry/solenoid_lock_electrical_connections.png" style="width:60%;" />
  <p class="figure-caption">Figure XX: Solenoid Lock Electrical Connections</p>
</div>

---

## Remote control as an interface with the subsystems
The RoboMaster remote control fulfils the RMUC competition requirement by allowing the gimbal operator to control the robot. Additionally, it serves as a practical tool for the Dart system development team to operate the robot during testing and calibration phases.
<div style="text-align:center;">
  <img src="/assets/images/barry/RoboMaster_Remote_Controller.png" style="width:60%;" />
  <p class="figure-caption">Figure XX: RoboMaster Remote Controller</p>
</div>


---

## Motors to actuate different mechanisms
For the initial prototype, the M2006 motor by DJI was selected due to its availability and future applicability. It provides a suitable platform for rapid prototyping, allowing demonstration of the robot’s expected behaviour and integration of the motor control code within the overall software architecture.
<div style="text-align:center;">
  <img src="/assets/images/barry/M2006_motor.png" style="width:60%;" />
  <p class="figure-caption">Figure XX: M2006 Motor</p>
</div>


---

## Future work
There are still some limitations in the current work such as:
  - The control system used to tension the elastic band to the correct force with accuracy and precision.
  - The code scalability with more task such as feeding, yaw and pitch. Consequently, the decision of priority for those tasks in the runtime of the program.
  - The load cell used is not rated for the actual design
  - Motor is not rated for the actual design
  - GUI library and touch function not implemented yet


