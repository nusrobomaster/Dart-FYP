---
title: Launcher - Electrical & Software
parent: Launcher Subsystem
nav_order: 5
layout: default
permalink: /launcher/elec&software
---

# Electrical / Software

## User journey

<p class="figure-caption" style="text-align:center;">
Figure XX: Video of User Journey
</p>

---

## Electrical and software requirements

From the user journey, five key functions were identified:

- Tension feedback for the launcher  
- Touch screen GUI  
- Electrical lock  
- Remote control  
- Motors for mechanisms  

---

## First critical function prototype

![Critical_function_prototype]({{ "/assets/images/barry/Critical_function_prototype.jpg" | relative_url }})
<p class="figure-caption" style="text-align:center;">
Figure XX: Critical Function Prototype
</p>

---

## Current electrical architecture

<div style="text-align:center;">
  <img src="{{ '/assets/images/barry/Segmented_power_diagram.png' | relative_url }}" style="width:60%;" />
  <p class="figure-caption">Figure XX: Segmented Power Diagram of Dart Robot</p>
</div>

<div style="text-align:center;">
  <img src="{{ '/assets/images/barry/Dart_robot_signal.png' | relative_url }}" style="width:60%;" />
  <p class="figure-caption">Figure XX: Dart Robot Signal Overview</p>
</div>

---

## Current software architecture

<div style="text-align:center;">
  <img src="{{ '/assets/images/barry/Data_flow_diagram.jpg' | relative_url }}" style="width:60%;" />
  <p class="figure-caption">Figure XX: Dart Robot Data Flow Diagram</p>
</div>

---

## Tension feedback for the launcher

<div style="text-align:center;">
  <img src="{{ '/assets/images/barry/Beam_type_load_cell_with_ADC.png' | relative_url }}" style="width:30%;" />
  <p class="figure-caption">Figure XX: Beam-type Load Cell with ADC</p>
</div>

---

## Touch screen GUI for entering inputs

### Touchscreen comparison

| Specification | TFT LCD 2.4" resistive | TFT LCD 3.5" resistive | Hosyond 3.5" IPS capacitive |
|--------------|-------------------------|--------------------------|------------------------------|
| Resolution   | 240×320                 | 480×320                  | 480×320                     |
| Touch        | Resistive               | Resistive                | Capacitive                  |
| Touch IC     | XPT2046                 | XPT2046                  | FT6336U                     |
| Display IC   | ILI9341                 | ILI9488                  | ST7796U                     |
| Communication| SPI                     | SPI                      | SPI                         |
| Cost         | $13.71                  | $17.85                   | $45.23                      |

---

## GUI Library Comparison

<div style="text-align:center;">
Table XX: GUI Library Comparison
</div>

---

## Electrical lock

<div style="text-align:center;">
  <img src="{{ '/assets/images/barry/solenoid_lock_electrical_connections.png' | relative_url }}" style="width:60%;" />
  <p class="figure-caption">Figure XX: Solenoid Lock Electrical Connections</p>
</div>

---

## Remote control

<div style="text-align:center;">
  <img src="{{ '/assets/images/barry/RoboMaster_Remote_Controller.png' | relative_url }}" style="width:60%;" />
  <p class="figure-caption">Figure XX: RoboMaster Remote Controller</p>
</div>

---

## Motors

<div style="text-align:center;">
  <img src="{{ '/assets/images/barry/M2006_motor.png' | relative_url }}" style="width:60%;" />
  <p class="figure-caption">Figure XX: M2006 Motor</p>
</div>

---

## Future work

- Tension control accuracy  
- Task prioritisation  
- Load cell upgrade  
- Motor upgrade  
- GUI + touch implementation remaining  
