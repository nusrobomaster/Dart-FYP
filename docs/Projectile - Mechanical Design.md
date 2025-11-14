---
layout: default
title: Projectile - Mechanical Design
parent: Dart Projectile
nav_order: 1
---

## 1. Foreground
The projectile is the primary payload of the DART system, and its design was a foundational focus of this project. The projectile's geometry and mass are critical system-level constraints that directly influence the design of the feeder mechanism and the launcher.

The project is governed by strict parameters from the competition organizer, RMOC, which define the maximum allowable size, weight, and power limits. However, the specific aerodynamic design of the projectile is unrestricted, requiring a rigorous development process involving iterative design and Computational Fluid Dynamics (CFD) analysis.

![Shenzhen Bay Sports Center Stadium](./assets/images/image_892365.jpg)
*Figure 1: The Shenzhen Bay Sports Center Stadium, the competition venue.*

![Actual images of the indoor arena setup, completely enclosed](./assets/images/image_8976fa.jpg)
*Figure 2: The enclosed competition arena, which has internal HVAC-induced crosswinds.*

A critical step before simulation was to establish the environmental base parameters. Misinterpreting these conditions, such as air density or wind speed, can invalidate CFD results.

The competition will be held at the Shenzhen Bay Sports Center Stadium, an enclosed, 48,210-square-meter arena. While NUS Calibur Robotics suggested this indoor location would have no wind, our team identified a significant risk of internal crosswinds. This was based on direct observation of large-scale HVAC (air conditioning) units during a 2024 site visit.

Therefore, despite the "enclosed" setting, a key design requirement was to ensure projectile stability in the presence of crosswinds, departing from the initial client assumption of a static-air environment.

!(./assets/images/Screenshot 2025-11-12 074207.jpg)
*Figure 3: Our DART System testing area at the NUS Engineering Auditorium.*

!(./assets/images/image_8981a0.jpg)
*Figure 4: Enclosed Multi-Purpose Sports Hall used for additional testing.*

For the design of the projectile, we decided to adopt a **first-principles design methodology**. This is a problem-solving approach that involves breaking down a complex issue into its most fundamental truths to build innovative solutions from the ground up.

We intentionally began with simple, passive prototypes. This was critical for us to learn the fundamental "pain points" of stability and drag, before we could justify adding the complexity of an active system. Our first prototypes were a rapid exploration of basic concepts.

### Iteration Path:

* **Passive projectile design:**
    * V1 (Thorn) → V3(Crossblade) ✅
* **Parallel investigation (Conceptual Detour):**
    * V2 (Glider) ❌
* **Active projectile design:**
    * V4(Xwing) → V5(Xwing 2) → V6(Trident) → V7(Hunter) ✅