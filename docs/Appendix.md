---
title: Appendix
nav_order: 10
layout: default
permalink: /appendix
---

# Appendix
<!-- Fill in according to sequence of mention in the report <-->
## Appendix XX: Calculations for torque requirements - Yaw
Calculations involved in the torque needed for yaw subsystem is detailed here.

<br>
![Yaw Torque Calculation 1](assets/images/jianwen/Appendix-yaw_calculation1.jpg)
{: .text-center}
<br>
![Yaw Torque Calculation 2](assets/images/jianwen/Appendix-yaw_calculation2.jpg)
{: .text-center}
<br>

## Appendix XX: Work done on belt driven system
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

## Appendix XX: Calculations for torque requirements - Pitch
Calculations involved in the torque needed for pitch subsystem for direct drive are detailed here.

<br>
![Pitch calculations 1](assets/images/jianwen/Appendix-pitch_calculation1.jpg)
{: .text-center}
<br>