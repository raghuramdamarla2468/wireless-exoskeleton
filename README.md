# Wireless Nurse-Assistive Exoskeleton

MSc Robotics Dissertation Project – Heriot-Watt University (2026)

## Project Overview

This project presents a fully wireless, sensor-driven wearable exoskeleton designed to reduce lower-back strain in nurses during patient-handling tasks.

The system integrates posture detection (BMI160 IMU) and load sensing (FSR sensors) using ESP32 microcontrollers and ESP-NOW wireless communication.

---

## System Architecture

- 1x Master ESP32 (Motor Control Unit)
- 1x BMI160 IMU Slave Node (Posture Detection)
- 2x FSR Sensor Slave Nodes (Load Detection)
- ESP-NOW Wireless Communication
- 12V DC Motor + IBT-2 Driver
- Independent Battery Powered Nodes

---

## Control Logic

Bending + Pressure → Motor Tighten  
Standing + No Pressure → Motor Release  

---

## Technologies Used

- ESP32
- ESP-NOW Protocol
- BMI160 IMU
- FSR Sensors
- Embedded C/C++
- Arduino IDE
- Wireless Sensor Networks
- Real-Time Control Systems

---

## Key Performance Results

- 97% Wireless Packet Reliability
- 93% Posture Detection Accuracy
- 96% Load Detection Accuracy
- 2–7 ms Communication Latency

---

## 📂 Complete Project Documentation

All detailed project materials including:

- 📄 Full MSc Dissertation Report  
- 📸 System Images & Circuit Diagrams  
- 🎥 Working Demonstration Videos  
- 💻 Complete Source Code  

are available here:

👉 [View Complete Project Files on Google Drive]
(https://drive.google.com/drive/folders/1dA3TuVWmqlDBgbl7_bq4trbJ_UqnaUIW?usp=sharing)

## Author

Raghuram Damarla  
MSc Robotics Graduate  
Heriot-Watt University
