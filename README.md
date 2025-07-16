# FPV Drone Flight Controller

This repository contains an experimental FPV drone flight controller firmware targeting the STM32F405 MCU. The code is written in C and provides a bare‑metal implementation with modules for hardware initialization, sensor handling, flight control and safety routines.

Current structure:

- `src/main.c` – main entry point and control loop
- `src/hardware_init.c` – system clock and peripheral setup
- `include/system_config.h` – global configuration and pin definitions
- `include/hardware_init.h` and `include/main.h` – basic headers

This is an initial skeleton based on design documents. Further modules such as motor control, receiver decoding and sensor fusion will be added later.
