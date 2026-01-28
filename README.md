# STM32 Sensor Fusion Application

This repository contains an embedded application for STM32 microcontrollers that integrates multiple sensors using I2C and FreeRTOS. The project demonstrates modular driver design, multitasking with queues, and structured data acquisition from environmental and inertial sensors.

## 📌 Overview

The application interfaces with the following sensors:

- **BMP280** – Temperature and pressure sensor  
- **MPU6050** – 3-axis accelerometer and gyroscope  
- **QMC5883P** – 3-axis digital magnetometer  

Each sensor runs in its own FreeRTOS task, and the acquired data is transmitted via UART in a structured format.

## 🧩 Features

- STM32 HAL-based drivers (I2C, UART, TIM)
- FreeRTOS multitasking architecture
- Independent sensor tasks
- Inter-task communication using queues
- Clean and modular driver implementation
- Portable and reusable code structure
- Suitable for sensor fusion and embedded research

## 🛠 Hardware Requirements

- STM32 microcontroller (tested with STM32F1 series)
- BMP280 sensor (I2C)
- MPU6050 IMU (I2C)
- QMC5883P magnetometer (I2C)
- USB-to-UART adapter (or onboard UART)
- External crystal (HSE), if required by the board

## 🧪 Software Requirements

- STM32CubeIDE
- STM32 HAL drivers
- FreeRTOS (CMSIS-OS)
- Git (optional, for version control)

## 📁 Project Structure

