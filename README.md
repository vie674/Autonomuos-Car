# 1:10 Autonomous Car Project

## 🏎️ Overview
The **1:10 Autonomous Car Project** is a scaled-down prototype designed to emulate real-world autonomous vehicle functionalities. This project focuses on applying robotics, machine learning, and embedded systems to create a self-driving car at 1:10 scale.

## ✨ Key Features
- **Autonomous Navigation**: Navigate a defined track using sensors and computer vision.
- **Obstacle Detection and Avoidance**: Utilize LiDAR, ultrasonic sensors, and cameras for obstacle handling.
- **Vehicle Control**: Precision steering and throttle control with PID algorithms.
- **RTOS Implementation**: Real-time operating system (RTOS) used for multitasking between perception, control, and communication systems.

## 🔧 Technology Stack
- **Hardware**: 
  - Raspberry Pi 4 for high-level processing.
  - STM32 microcontroller for real-time control.
  - LiDAR, cameras, and ultrasonic sensors for environment sensing.
- **Software**:
  - Python and C++ for control algorithms.
  - OpenCV for image processing and computer vision.
  - TensorFlow for machine learning-based lane detection.
  - RTOS for scheduling critical tasks.

## 🛠️ System Architecture
The project is divided into three main subsystems:
1. **Perception**: Collects and processes data from sensors (LiDAR, camera, etc.).
2. **Planning**: Determines the path and avoids obstacles.
3. **Control**: Executes throttle, steering, and braking commands.

![System Architecture](system_architecture.png)

## 🚀 Getting Started
### Prerequisites
- Raspberry Pi 4 with Raspbian OS.
- STM32 development board.
- Compatible sensors (LiDAR, cameras, etc.).
- Python 3.x and required libraries (see `requirements.txt`).

### Installation
1. Clone this repository:
   ```bash
   git clone https://github.com/yourusername/1-10-autonomous-car.git
   cd 1-10-autonomous-car
