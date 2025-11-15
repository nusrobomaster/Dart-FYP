---
title: 4 Mechanical Design – Pitch
parent: Dart Robot
grand_parent: Dart System
nav_order: 1      # after “Dart Robot - Mechanical Design” and “Dart Robot - Electrical & Software”
layout: default
permalink: /dart-system/dart-robot/pitch
has_toc: true
---

## Mechanical: Pitch
The pitch subsystem controls the upper assembly’s inclination and supports the launcher and feeder. It also includes a secondary load-bearing component discussed later.
### Load consideration
The key load factor is the torque required to rotate the 15 kg upper assembly about its axis. Torque calculations for each drive system are detailed in the Drive System Considerations section.
### Drive system considerations
The drive system must:
1. Provide sufficient torque to rotate the loaded upper assembly
2. Hold its position at the desired inclination
3. Achieve precise and accurate angle control

Based on these requirements, two drive systems were shortlisted (Table ??).

<br>
![Table for drive systems](/assets/images/jianwen/Pitch-drive_system_table.jpg)
{: .text-center}
<br>
<p align="center" class="small-text">
<em>Table ??: Drive System Considerations</em>
</p>
Design work for both direct-drive and leadscrew systems is ongoing. The team currently leans toward a direct-drive setup with reduction gear due to its lower mechanical complexity and higher reliability, though the final choice will be confirmed in the next phase.

Torque calculations for the direct-drive design assumed a 15 kg load acting at 0.75 × assembly length. At 25° inclination, the perpendicular distance is greatest, producing the maximum moment. This value represents both the required and holding torque. Full workings are in Appendix ?. The following equation was used:

<br>
![Equations for torque](/assets/images/jianwen/Pitch-equation1.jpg)
{: .text-center}
<br>
The resulting torque is 110.3 Nm.

Using the following relation, the motor’s required input torque was determined:

<br>
![Equations for reduction gear](/assets/images/jianwen/Pitch-equation2.jpg)
{: .text-center}
<br>
With a 1:50 gearbox at 70% efficiency, the required motor torque is 3.2 Nm.
### Design consideration: Load-bearing assistive component
If a direct-drive system is selected, a load-bearing assist component will be added to support the upper assembly. This reduces motor wear and avoids the need for a high-torque brake.  
### Next steps
Further research, sourcing, and team discussions will finalise the pitch drive system and motor. Mounting interfaces for the launcher and feeder will be completed in the project’s second phase.
