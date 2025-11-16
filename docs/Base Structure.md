---
title: 8 Base Structure
nav_order: 3
layout: default
permalink: /base_structure
---
<!-- nav_order is for DROPDOWN NAVIGATION sequence <--> 
# Base
The base structure is a key objective in both attack and defense. Destroying the opponent’s base wins the match. In our project, it is one of two designated targets for the dart robot and projectile. The team will design and fabricate the base as outlined below.

Two iterations are planned:
1. First iteration: a test target for Dart launcher and projectile subsystems.
2. Second iteration: a close replica of the competition base for realistic testing.

As the official base cannot be purchased, an in-house version will be built to support testing of both the launcher and projectile subsystems. Both versions will also be used by the Calibur Robotics team for practice.

## Base: Design Specifications
<br>
<p align="center">
  <img src="{{ '/assets/images/vj/Base/Base_Design_Specificcations_Table.png' | relative_url }}" width="600">
</p>
{: .text-center}
<p align="center" class="small-text"><em>Table 8-1: Base_Design_Specificcations_Table</em></p>

## Base: Passive structure
First iteration will include the competition’s essential base components. It will be built quickly as a reliable target for the dart robot and projectile, and for other NUS Calibur Robotics robots.
### Design process: Structure
The base structure was designed to resist impact damage from various projectiles. Due to the short timeline, a proven structural style was adopted. Inspired by the Eiffel Tower, the base uses a truss design—wider at the bottom and narrower at the top (Figure 8-1). All trusses are made from 2 mm 6061 aluminum hollow square tubing, joined with 3 mm 6061 aluminum gusset plates for alignment and reinforcement.

<br>
![Main Structure](assets/images/jianwen/Base-main_structure.jpg)
{: .text-center}
<br>
<p align="center" class="small-text">
<em>Figure 8-1: Truss structure making up main body of base</em>
</p>
The back support trusses connecting the base frame to the mid-section are angled and aligned with the expected impact direction of the dart. This prevents structural instability under diagonal impact of the projectile.

### Design Process: Guidance Light Source and cooling system

<br>
Light and Fans are added to the Dart Detection Module.
<br>
<p align="center">
  <img src="{{ '/assets/images/vj/Base/Light_Specificcation_Table.png' | relative_url }}" width="450">
</p>
{: .text-center}
<p align="center" class="small-text"><em>Table 8-2 : Light Specifications Table</em></p>
<br>
<div style="text-align: center;">
<iframe width="560" height="315" src="https://www.youtube.com/embed/mlMt_C0wRAE?si=Pii15hxr6KVMaU5U" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
</div>
<p align="center" class="small-text"><em>Video 8-1 Temperature Test for Dart Guiding Light</em></p>
<br>
Fans are required as the light goes from 40°C to 100°C in 1 minute without cooling, and the temperature keeps rising. It takes about eight minutes to cool back to room temperature.
<br>

<p align="center">
  <img src="{{ '/assets/images/vj/Base/Fan_Specifications_Table.png' | relative_url }}" width="450">
</p>
{: .text-center}
<p align="center" class="small-text"><em>Table 8-3 : Fan Specifications Table</em></p>
<br>

The fans have shown to be very effective in cooling the light as seen below.

<br>
<p align="center">
  <img src="{{ '/assets/images/vj/Base/Temp_Test_Dart_Green_Light.png' | relative_url }}" width="600">
</p>
{: .text-center}
<p align="center" class="small-text"><em>Table 8-4 : Comparisons of the Light Temperature between NO FAN and WITH FAN conditions table</em></p>
<br>

### Design process: Light indicator component
The base includes a dart target that slides linearly over 980 mm and is mounted 1300 mm above ground on the truss top. A green light, as specified in the competition rules, guides the dart toward the target. Above it, an armor plate inclined at 27° detects hits.

Two design considerations were addressed:
1. The linear motion mechanism.
2. A unified mount for the light, cooling system, and armor plate.

To simplify assembly, all components are fixed to a single 3D-printed mount attached to a linear rail and carriage. Figure 8-2 shows the mount design, and Figure 8-3a & 8-3b shows the CAD model and fabricated component respectively.

<br>
![3D printed component](assets/images/jianwen/Base-Light_Component.jpg)
{: .text-center}
<br>
<p align="center" class="small-text">
<em>Figure 8-2: 3D printed mounting piece CAD</em>
</p>

<br>
![Sliding component](assets/images/jianwen/Base-sliding_component.jpg)
{: .text-center}
<br>
<p align="center" class="small-text">
<em>Figure 8-3a and 8-3b: Fully assembly CAD, (a) and fabricated assembly (b)</em>
</p>

### Design process: Mounting of armor components
At the client’s request, two extra armor plates were added for use by other Calibur Robotics robots. They are mounted at 487 mm and 1058 mm above ground, with the upper plate inclined 27° as per competition rules.

The mounts use the same truss design, but with 3D-printed TPU pieces between the frame and armor plates to absorb shock from repeated impacts. Figures 8-4a and 8-4b show the TPU mount in CAD and after installation.

<br>
![Armor Mounting](assets/images/jianwen/Base-armor_plate_mount.jpg)
{: .text-center}
<br>
<p align="center" class="small-text">
<em>Figure 8-4a and 8-4b: TPU mounting piece for top large armor plate CAD, right (a), and fabricated, left (b)</em>
</p>
Figure 8-5 shows the full assembly of the passive base structure with both large armor plates mounted.

<br>
![Full CAD](assets/images/jianwen/Base-full_structure_cad.jpg)
{: .text-center}
<br>
<p align="center" class="small-text">
<em>Figure 8-5: Full CAD of the passive base structure</em>
</p>

### Fabrication, Assembly, and Testing
All trusses, plates, and gussets were fabricated with support from the NUS Central Workshop using standard request procedures. 

Assembly took place in the same workshop, with minor truss adjustments made to correct fabrication errors. These did not delay progress. The completed setup revealed a useful “self-locking” feature in the linear rail, which holds the slider in place after movement (Figure 8-6).

<br>
![Fabricated Structure](assets/images/jianwen/Base-fabricated_structure.jpg)
{: .text-center}
<br>
<p align="center" class="small-text">
<em>Figure 8-6: Full assembled passive base structure</em>
</p>
A dummy dart projectile of the required weight was thrown at the structure five times. The base remained stable with no visible damage or component failure.

### Next steps
The next phase involves designing and fabricating the second base iteration with added components to better simulate competition conditions. These include motorized linear motion for target randomization, movable guard panels for the lower plate, and an ESP32 web server for remote control via user devices.

Below are the tentative Signal and Power diagrams for the Base.

<br>
<p align="center">
  <img src="{{ '/assets/images/vj/Base/Tentative_Base_Signal_Diagram.png' | relative_url }}" width="600">
</p>
{: .text-center}
<p align="center" class="small-text"><em>Figure 8-8 : Tentative Segmented Base Signal Diagram</em></p>
<br>

The ESP32 is set to act as an access point that other devices can connect to. Users can connect to the Web Server via their Laptops/Phones via the webserver IP address and then control the state of the base components. 

The motor driver is to control the linear motion of the armour module & light, Relay 1 and 2  is to turn on/off the fans and the light respectively.

<br>
<p align="center">
  <img src="{{ '/assets/images/vj/Base/Tentative Base Power_Diagram_Segmented.png' | relative_url }}" width="600">
</p>
{: .text-center}
<p align="center" class="small-text"><em>Figure 8-9 : Tentative Segmented Base Power Diagram</em></p>
<br>

A 24V battery similar to the ones used throughout Calibur Robotics is used as the main power source. 
It supplies the Guiding Light and the motor and is stepped down to power an ESP32, relays and the fans. 

<br>
<p align="center">
  <img src="{{ '/assets/images/vj/Base/Tentative_Wire_Frame_of_ESP32_WebServer.png' | relative_url }}" width="600">
</p>
{: .text-center}
<p align="center" class="small-text"><em>Figure 8-10 : Tentative Wire Frame of ESP32 WebServer</em></p>
<br>

The above wireframe shows the UI on the user device.