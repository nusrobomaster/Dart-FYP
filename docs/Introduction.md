---
title: Introduction
nav_order: 1
layout: default
permalink: /overview
---

# Introduction
### Abstract
This project aims to develop a projectile launcher and projectile to hit targets in the RoboMaster University Championship (RMUC). To do so, ASI - 403 would be building an accurate mock - up of the target for testing, implementing in-flight corrective functionality into the projectile and adjustable pitch & yaw and auto projectile reloading for the launcher. Predictive Maintenance and easy debugging & maintenance is prioritized.
## Overview of RoboMaster University Championship (RMUC)

Founded in 2013, DJI RoboMaster University Championship (RMUC) is an annual international robotics competition organized by Da-Jiang Innovations (DJI) in China. Teams would develop different robots to perform combat on the battlefield. 

<br>
![Battlefield](assets/images/vj/Intro-battlefield.jpg)
{: .text-center}
<br>
<p align="center">
<em>Figure XX: Top view of the battlefield</em>
</p>
The goal is to launch projectiles at the pressure sensors of opponent robots and targets to defeat them.

## Targets

<br>
![Battlefield Targets](assets/images/vj/Intro-battlefield_targets.jpg)
{: .text-center}
<br>
<p align="center">
<em>Figure XX: Battlefield with Labelled Target Locations</em>
</p>

<br>
![Target Table](assets/images/vj/Intro-target_table.jpg)
{: .text-center}
<br>
<p align="center">
<em>Table XX: Target Pressor Sensor Description</em>
</p>

## Winning Criteria

<br>
![Base and outpost](assets/images/vj/Intro-base_outpost.jpg)
{: .text-center}
<br>
<p align="center">
<em>Figure XX: Close up of Targets</em>
</p>
A team wins if the opponent's base HP reaches zero OR its base has more remaining HP than the opponent’s at the end of the match. Base is invincible until outpost is destroyed.

## Armour Modules

<br>
![Base armors](assets/images/vj/Intro-base_armors.jpg)
{: .text-center}
<br>
<p align="center">
<em>Figure XX: Base with Small and Large Armour Modules</em>
</p>
Most robots and targets, such as Base, use armor modules — official game system components with pressure sensors—to register hits which decreases health.

<br>
![armor parts](assets/images/vj/Intro-dart_armor.jpg)
{: .text-center}
<br>
<p align="center">
<em>Figure XX: Close up of Dart Detection Module which is on the Targets (Outpost & Base)</em>
</p>

## Robot Roles

<br>
![Robot lineup Table](assets/images/vj/Intro-robot_lineup_table.jpg)
{: .text-center}
<br>
<p align="center">
<em>Table XX: The current NUS Calibur Robotics robot lineup</em>
</p>
ASI - 403 would be building a new DART robot. (See Appendix XX for description of other robots)

## Launch Window Mechanics

<br>
![Dart flight](assets/images/vj/Intro-dart_arc.jpg)
{: .text-center}
<br>
<p align="center">
<em>Figure XX: Example Projectile Flight Path</em>
</p>
The robot holds up to four projectiles and stays inside the Dart Launching Station with a gate that can open twice per round—once after 30 seconds and again after 4 minutes from the start of the round. Each opening lasts 20 seconds, followed by a 15-second cooldown. Unused openings can be used after the 4 minute mark.

The operator selects which target (Outpost/Base) the Dart Robot aims at each opening. The team wants us to aim at base.(Strategy explained in Appendix XX)

## Base Targeting Mechanics
Teams can select the desired Base target difficulty. 

<br>
![Difficulty Table](assets/images/vj/Intro-difficulties.jpg)
{: .text-center}
<br>
<p align="center">
<em>Table XX: Base Target Difficulty Levels</em>
</p>
Higher difficulty levels increase the damage dealt and debuffs applied to the opponent, while granting greater buffs to the attacking team upon a successful hit. (See Appendix XX for the damage mechanisms and the buff & debuff mechanisms)

# Past attempts’ challenges and our improvements
Since 2022, NUS Calibur Robotics has made several attempts to develop a dart robot and projectile capable of accurately hitting competition targets.
## Dart Projectile
Earlier designs suffered from unstable flight due to weak aerodynamic choices (Figure ??). This guided our new approach, which focuses on fin geometry, airflow behaviour, and roll control. The projectile will also be upgraded to an active design capable of adjusting its trajectory toward the target.

<br>
![Past Projectiles](assets/images/jianwen/Intro-past_dart_projectile.jpg)
{: .text-center}
<br>
<p align="center">
<em>Figure ??a (left) and Figure ??b (right): Past attempts at the dart projectile</em>
</p>

## Dart Robot
Past robots showed unreliable pitch–yaw control, base instability, and inconsistent release (Figure ??). These issues shaped the new design, which prioritizes stable actuation and a stronger frame. The flywheel launcher will be replaced with an elastic band system for more consistent release and improved accuracy.

<br>
![Past Robots](assets/images/jianwen/Intro-past_dart_robot.jpg)
{: .text-center}
<br>
<p align="center">
<em>Figure ??a (left) and Figure ??b (right): Past attempts at the dart robot</em>
</p>

## Steps forward
Insights from earlier attempts guide our integrated redesign. The projectile will be developed first, with improved aerodynamics and active-flight components. The robot will follow with reliable pitch–yaw control, a stable frame, and a simplified launch mechanism to support accurate, repeatable shots.
