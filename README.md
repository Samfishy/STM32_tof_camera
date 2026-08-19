# STM32 ToF Thermal-Style Depth Camera 📸

This project implements a low-cost, real-time 3D depth camera that visualizes spatial distance similar to how a thermal camera visualizes heat. 

Built around the **STM32F411** microcontroller and the **VL53L5CX Time-of-Flight (ToF) sensor**, the system captures an 8x8 or 4x4 distance matrix. It then uses **ARM CMSIS-DSP** fixed-point math to bilinearly upscale the coarse data into a smooth, high-resolution grid. The resulting data is mapped to a 30-color gradient and rendered as a live heatmap on a 1.8" TFT display using **LVGL**.

## ✨ Features
* **Live Heatmap Visualization**: Converts 3D spatial distances into a smooth color-coded heatmap in real-time.
* **Dynamic Resolution & Scaling**: Switch between 8x8 (15Hz) and 4x4 (60Hz) zone modes on the fly, with 1x, 2x, or 4x bilinear interpolation.
* **Non-Volatile Recording**: Capture and save depth frames directly to an external W25Qxx SPI Flash memory.
* **Playback Mode**: Browse and review saved distance captures directly on the device with interactive paging.
* **Professional UI**: Powered by LVGL 8.4, featuring dynamic loading screens, smooth transitions, and live on-screen variable tracking.

## ⚙️ Software Architecture (V2.0)
The software stack was recently overhauled from a bare-metal polling architecture to a high-performance RTOS system:
* **FreeRTOS**: Manually configured (without CubeMX) to handle system concurrency. Heavy workloads (Flash I/O, display rendering) are offloaded to prioritized tasks, slashing ISR execution time.
* **DMA-Driven I/O**: Both the VL53L5CX ToF Sensor (I2C) and the ST7735 TFT Display (SPI) utilize DMA. This asynchronous data flow frees up CPU cycles for DSP upscaling and dramatically boosts framerates.
* **CMSIS-DSP**: Utilizes Q15 fixed-point bilinear interpolation with strict boundary clamping for artifact-free image upscaling.
* **Memory Optimized**: By rendering directly to LVGL canvases and saving raw sensor data instead of heavy bitmaps, RAM footprint and Flash storage are highly optimized.

## 🛠️ Hardware Requirements
* **MCU**: STM32F411CEU6 ("Black Pill") 
* **Sensor**: STMicroelectronics VL53L5CX (Multizone ToF)
* **Display**: 1.8" ST7735 SPI TFT LCD (160x128)
* **Storage**: Winbond W25Qxx SPI NOR Flash (e.g., W25Q64)
* **Input**: 4x Tactile Push Buttons

## 🔌 Pinout & Wiring

| STM32 Pin | Peripheral | Description |
| :--- | :--- | :--- |
| **I2C2 (Sensor)** |
| `PB10` / `PB3` | VL53L5CX | I2C SCL / SDA |
| `PB7` | VL53L5CX | Reset Pin (`rst_tof`) |
| `PB8` | VL53L5CX | Low Power Control (`lp_tof`) |
| `PB6` | VL53L5CX | Ranging Data Ready Interrupt (EXTI6) |
| **SPI4 (Display)** |
| `PA9` | ST7735 | DC (Data / Command) |
| `PB1` | ST7735 | CS (Chip Select) |
| `PB2` | ST7735 | RES (Reset) |
| **SPI1 (Flash)** |
| `PB0` | W25Qxx | CS (Chip Select) |
| **User Inputs** |
| `PA0` | Button 1 | EXTI0 (Interpolation toggle / Next image) |
| `PA2` | Button 2 | EXTI2 (LIVE / REC mode switch) |
| `PA3` | Button 3 | EXTI3 (Zone toggle / Prev image) |
| `PA4` | Button 4 | EXTI4 (Capture frame) |

## 🚧 Current Challenges & To-Do
- [ ] **Hardware Button Debouncing**: Currently implementing software latching (disabling EXTI inside the ISR and deferring to an RTOS task). Exploring more robust FreeRTOS-friendly debouncing strategies to prevent edge-case system freezes.
- [ ] **PCB Design**: Move from breadboard/perfboard to a custom printed circuit board.

## 📄 License
This project is open-source. Feel free to use, modify, and distribute it as you see fit.
