---
title: 10 Appendix
nav_order: 10
layout: default
permalink: /appendix
---

# Appendix
<!-- Fill in according to sequence of mention in the report <-->
## Appendix 1: Roles of Robots in the lineup other than Dart
<br>
Some robots perform an attacking function whereas others like engineer or radar perform a more support function like providing enemy robot location or picking up boxes. 
<br>
<p align="center">
  <img src="{{ '/assets/images/vj/Robot_Roles.png' | relative_url }}" width="600">
</p>
{: .text-center}
<p align="center" class="small-text"><em>Robot Roles</em></p>

## Appendix 2: Team’s Strategy for focus on Base
<br>
<p align="center">
  <img src="{{ '/assets/images/vj/team_strategy.png' | relative_url }}" width="600">
</p>
{: .text-center}
<p align="center" class="small-text"><em>Captain's Reasoning for Team's Strategy for dart system  to target Base</em></p>

## Appendix 3: Damage Mechanism and the Buff & Debuff mechanisms
<br>
The below buffs , damages and debuffs are summarised from page 75 of the Robomaster 2025 University Championship Rules Manual
<br>
### Damage Mechanism
<br>
<p align="center">
  <img src="{{ '/assets/images/vj/Comparison_Projectile_Damage_Targets.png' | relative_url }}" width="600">
</p>
{: .text-center}
<p align="center" class="small-text"><em>Comparison of projectile damage to Targets (Base & Outpost)</em></p>
<br>
Dart deals more damage than other projectiles. 

### Attacking Team Buffs and Opposing Team Debuffs upon successful Dart hits
<br>
<p align="center">
  <img src="{{ '/assets/images/vj/RMUC_Terminologies_Dart_Buffs_Debuffs.png' | relative_url }}" width="600">
</p>
{: .text-center}
<p align="center" class="small-text"><em>RMUC Terminologies for Dart Buffs and Debuffs</em></p>
<br>
<br>
<p align="center">
  <img src="{{ '/assets/images/vj/Buffs_and_Debuffs_Table.png' | relative_url }}" width="600">
</p>
{: .text-center}
<p align="center" class="small-text"><em>Buffs and Debuffs table for varying targets and settings</em></p>
<br>
<br>

## Appendix 5: Torque Calculations
<br>
Torque, T is minimum torque required to move the servo horn with the fin attached against gravity. 
Weight of Horn and Fin, F is the mass x gravitional force = weight that gets acted on the horn and fin by Earth
Distance, d is the distance from the centre of the servo horn to the top of the fin


\
T = Fd


\
  = 2.3g x 15.2mm


\
  = (2.3 x 10^{-3})(9.81)(15.2 x 10^{-3})


\
  = 3.4295 x 10^{-4}N·m


## Appendix 6: PCB Schamtic and BOM

### PCB Schematic





## Appendix 7: Calculations for torque requirements - Yaw
Calculations involved in the torque needed for yaw subsystem are detailed here.

<br>
![Yaw Torque Calculation 1](assets/images/jianwen/Appendix-yaw_calculation1.jpg)
{: .text-center}
<br>
![Yaw Torque Calculation 2](assets/images/jianwen/Appendix-yaw_calculation2.jpg)
{: .text-center}
<br>

## Appendix 8: Work done on belt driven system
Further details of the belt-driven system are as follows.

The following will be what we need for the belt drive components, less the mounting components
- Timing gear x2, 1 driving 1 idle
- Tensioner x1
- Timing belt x1, to the appropriate length

We will look into the considerations first

**Considerations for belt drive components**
Firstly the timing gears and the corresponding belt, the following are some types we can look to use
1. GT2 —> pitch 2mm
  - For use in high precision, low torque use cases
2. HTD 3M —> pitch 3mm
  - Balanced torque & smoothness, common in most medium yaw systems
3. HTD 5M —> pitch 5mm
  - For high torque use cases
  - Significant backlash, but higher strength

For our launcher, with an expected axial load of 10-15kg, our system requires considerable torque to rotate, meaning that we are unable to proceed with the GT2 type, as much as we require a high precision setup.

We will also be eliminating the HTD 5M typing due to the significant potential backlash present in the setup system. The amount of torque we need to rotate the system will also not approach a very large amount, hence we no not need the additional strength that the HTD 5M type will provide.
With these reasons, we will be moving forward with the HTD 3M type timing gear and belt.

**Considerations for the tensioner**
The following are some possible types of tensioners for belt driven systems
1. Fixed idler —> Used when center distance is adjustable
  - Cheap and simple to implement
2. Spring Tensioner —> Used in most yaw systems, seen in cars
  - Auto adjusts, good for vertical setups
3. Cam Tensioner —> Adjustable with hex key
  - Usually used for compact setups

Another idea for tensioner:

<br>
![Yaw belt tensioner](assets/images/jianwen/Appendix-beltdrive_tensioner.jpg)
{: .text-center}
<br>
Explanation - simple terms, a screw will screw inwards along threads on the moving block. The screw shown by the blue arrow will serve as a “block”, which will stop the screw being screwed inwards to stop moving inwards. This in turn causes the moving block to move backwards instead. This pulls the belt apart, achieving tension.

Since we can machine out of metal, and machining is not very complicated. No need for extra tensioning pulley and what not as well. This idea references a compact tensioner found in a robot arm application. Reference document below, truncated to pages 127 - 131 of the original source
Components selected:
1. Timing gear-Yaw: HTD3M 90T Timing Gear
2. Timing gear-Motor: HTD3M 30T Timing Gear
3. Timing Belt: HTD3M 233T 699mm length belt (required 696.5mm)

## Appendix 9: Calculations for torque requirements - Pitch
Calculations involved in the torque needed for pitch subsystem for direct drive are detailed here.

<br>
![Pitch calculations 1](assets/images/jianwen/Appendix-pitch_calculation1.jpg)
{: .text-center}
<br>