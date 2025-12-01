# 🔐 Solenoid Lock Control (Remote Subproject)

This subproject contains the STM32 firmware for controlling an **electronic solenoid lock** via a **remote controller**. It is part of the dart launcher FYP under the RoboMaster competition. The system allows the user to safely load and release a lock latch using a dedicated switch on the controller.

## 🎮 Control Scheme

The **top-left switch** on the remote has two functions:

- **Middle position → Loading Mode**  
  The solenoid lock is engaged, allowing the player/user to **safely load the lock latch** into place.
  
- **Bottom position → Unlock Mode**  
  The solenoid lock is released, freeing the latch and completing the firing cycle.

## 🔄 Example Use Case

1. User sets the controller’s **top-left switch to the middle position** → system enters **Loading Mode**.  
2. User inserts the lock latch into the solenoid lock.  
3. User sets the controller’s **top-left switch to the bottom position** → solenoid **releases the latch**.  
4. To repeat, the user returns the switch to the middle position and reloads.  

## ⚙️ Technical Notes

- Built for **STM32CubeIDE (DEVA environment)**.  
- Solenoid lock is controlled via a GPIO output pin (uses PA0 for the detection of lock and PA1 for the control of lock)
- Remote controller input is decoded via the communication interface 

## 📸 Hardware Connections

- **Solenoid Lock Wiring**  
  ![Solenoid Lock Connection](./solenoid.webp)

- **Connection to DEVA Board**  
 ![DEVA Board Connection](./wiring_solenoid.jpg)

---

💡 This subproject is a modular component of the larger dart launcher system, ensuring **safe loading** and **controlled release** during RoboMaster operations.  
