1:10 Scale Self-Driving Car
Project Overview
This project involves the design and development of a 1:10 scale self-driving car. The car is equipped with various sensors and control systems to enable autonomous navigation, including lane detection, speed control, and real-time communication with a simulation cockpit over Wi-Fi using UDP. The project integrates both hardware and software components, including STM32 microcontroller for motor control, Raspberry Pi for image processing, and various sensors for feedback.

Features
Autonomous Driving: The car is capable of navigating autonomously by detecting lanes and adjusting speed based on the environment.
Real-Time Communication: Real-time control and feedback from the car to a simulation cockpit via UDP over Wi-Fi.
Sensor Integration: IMU for orientation feedback, encoders for speed and steering angle, and potentiometer for throttle input.
Precise Control: Use of STM32 for motor and steering control, optimized for low-latency responses.
Hardware Components
Raspberry Pi: Acts as the central processor for image processing and control decision-making.
STM32 Microcontroller: Handles motor and servo control based on input from sensors and instructions from Raspberry Pi.
DC Motors and Servo Motors: Control the movement and steering of the car.
IMU (Inertial Measurement Unit): Provides orientation data for the car.
Encoder: Measures the rotation speed and steering angle.
Potentiometer (Gas Pedal): Used for throttle control via an analog-to-digital conversion (ADC) on the STM32.
Software and Tools Used
Programming Languages: C (for STM32), Python (for Raspberry Pi)
Libraries/Frameworks: OpenCV (for lane detection and image processing), UDP (for communication)
Embedded Development Environment: Keil uVision (for STM32 programming)
Communication Protocol: UDP for real-time data exchange between the car and the simulation cockpit.
How It Works
Lane Detection: The Raspberry Pi processes camera images using OpenCV to detect the lane and adjust the steering angle.
Motor Control: The STM32 receives control signals from the Raspberry Pi to adjust the speed and steering of the car, based on sensor feedback (IMU, encoder).
Throttle Control: The throttle input from the gas pedal is read via an ADC on STM32, which adjusts the speed of the car accordingly.
Real-Time Data Exchange: The car sends real-time speed and steering data to the simulation cockpit via UDP, allowing remote control and monitoring.
