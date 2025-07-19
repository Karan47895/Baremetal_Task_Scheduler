# 🕹️ Bare-Metal Task Scheduler on STM32F407

In this project, I have implemented a task scheduler for managing four tasks on the **STM32F407VG Discovery board** using **bare-metal programming**. The scheduler is designed to handle sensor data acquisition from multiple sensors and transmit the data to an **ESP32** module for further processing.

### 📦 Sensors Used:
- **Light Sensor** – Measures ambient light intensity.  
- **Sound Sensor** – Detects sound levels.  
- **Ultrasonic Sensor** – Measures distance using ultrasonic waves.  
- **Temperature and Humidity Sensor** – Monitors environmental temperature and humidity.

The task scheduler ensures that each sensor is polled periodically in a **round-robin** fashion. The sensor data is then transmitted from the STM32 to the ESP32 module via the **UART protocol**. Once the ESP32 receives the data, it establishes a **Wi-Fi connection** and uploads the readings to a **Firebase database**, making the sensor data available for **remote monitoring and analysis**.

---

## ✨ Key Features

- **🛠️ Bare-metal Programming:**  
  The system is developed without any operating system, relying on direct hardware control to schedule tasks.

- **📈 Sensor Data Collection:**  
  Periodic reading of four sensor types, ensuring real-time data capture.

- **🔄 UART Communication:**  
  Efficient transmission of sensor data from the STM32 to ESP32.

- **☁️ Firebase Integration:**  
  The ESP32 sends the collected sensor data to a Firebase database over Wi-Fi, enabling cloud-based storage and visualization.

---

## 📌 Summary

This project demonstrates **efficient real-time sensor data handling** and **communication between embedded systems**, leveraging both hardware and software resources optimally for **IoT-based applications**.
