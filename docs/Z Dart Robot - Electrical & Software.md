---
title: 7 Launcher - Electrical & Software
parent: Dart Robot
grand_parent: Dart System
nav_order: 4
layout: default
permalink: /dart-system/dart-robot/elec-software
---

# Electrical / Software

## User journey


<div style="text-align: center;">
<iframe width="560" height="315" src="https://www.youtube-nocookie.com/embed/FLCwOA101J8?si=7lEBV499AkXXTPhk" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
</div>

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

| Criteria                          | <img src="{{ '/assets/images/barry/LVGL_logo.png' | relative_url }}" width="140"> | <img src="{{ '/assets/images/barry/Touch_GFX_logo.png' | relative_url }}" width="140"> | <img src="{{ '/assets/images/barry/QT_logo.png' | relative_url }}" width="140"> |
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
