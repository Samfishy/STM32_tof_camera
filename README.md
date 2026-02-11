ToF Camera with STM32 – Embedded Image Processing System
Overview
This project implements a Time-of-Flight (ToF) camera system using an STM32 microcontroller.  
The system captures frames from the ToF sensor, performs interpolation, displays the processed image, and allows storing and viewing captured frames from external QSPI flash memory.

The design focuses on real-time processing, simple gallery navigation, and configurable zone and interpolation modes.

Features
- Capture image frames from a ToF camera.
- Real-time processing on STM32.
- Interpolation and color mapping.
- Live display output on a TFT display.
- Capture button to:
  - Store current frame in QSPI flash.
  - Automatically switch to gallery mode.
- Gallery navigation:
  - Browse stored images.
  - Maximum storage: **10 images**.
- Selectable sensing zones:
  - 4×4
  - 8×8
- Adjustable interpolation:
  - 1×
  - 2×
  - 4×

Hardware Requirements
- STM32 microcontroller with 512 kb of flash.
- VL53L5CX.
- External SPI flash memory.
- TFT Display .
- User input buttons:
  - Capture
  - interpolation value
    zone select
  - Mode select

