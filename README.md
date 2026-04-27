# vivtech
**EECS 373: Smart Vivarium Control System**

EECS 373 at University of Michigan | Winter 2026

**Core Contributors:** 
Megha Jacob, Henry Springsteen, David Jackson, Dominic Duncan

**Project Description:** 
The Smart Vivarium is a high-reliability, bio-integrated embedded system designed to maintain precise environmental conditions for a bearded dragon. It features customizable conditions including temperature and humidity. This system utilizes closed-loop PID control and multi-layered safety interlocks to prevent thermal runaway and ensure occupant safety.

**Technical Highlights:**
-Custom 12V 10A PCB with buck converter, LDO, and low-side MOSFET switching for GPIO control of heater/fan, humidifer
-Custom watchdog file that monitors temperature to check if it is within safety limits and shuts down the heater if not
-Custom PID algorithm for heater control
-Custom LCD screen and touch screen drivers
-Arducam integration with the system

This repository includes all drivers, UI, CAD, and PCB files associated with the project

tjpgd.c and .h, and tjpgdcnf.h are open sourced from: https://elm-chan.org/fsw/tjpgd/

all other code is team-built.

Collaborators: Megha Jacob, Dom Duncan, David Jackson, Henry Springsteen
