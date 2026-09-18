### GPS Autonomous Wheel Robot with Real-Time Obstacle Avoidance

## Problem Statement

Traditional outdoor mobile robots often struggle to reach target locations with reliable accuracy and smooth motion when relying solely on basic GPS. The challenge addressed in this project was to design a wheel-steering robot capable of autonomously navigating to a designated GPS coordinate with position accuracy better than 3 meters, while maintaining smooth steering control and actively avoiding obstacles in real time.
Solution Overview

The system combines precise positioning, heading estimation, closed-loop control, and remote communication to deliver reliable outdoor navigation:

## Positioning & Heading
GPS module (Neo-6M) provides absolute location.

QMC5883L digital magnetometer (compass) supplies real-time heading.

Haversine formula calculates both the remaining distance to the target and the required bearing angle.

## Motion Control
PID controller continuously adjusts the steering wheel to minimize heading error, resulting in smooth and stable path following.

## Obstacle Avoidance
Ultrasonic sensors detect nearby obstacles and trigger real-time path corrections.

## Communication & Remote Operation
ESP32 microcontroller orchestrates all sensors and actuators.

MQTT protocol enables remote monitoring and control from a dashboard or mobile device.

## Hardware Stack

ESP32 · GPS Neo-6M · QMC5883L Magnetometer · Ultrasonic Sensors · Motor Drivers · MQTT over Wi-Fi
Key Technical Achievements

Sub-3 m arrival accuracy under outdoor conditions

Smooth closed-loop steering via PID

Real-time obstacle detection and reaction

Full remote telemetry and control through MQTT

## Skills & Competencies Demonstrated

Multi-sensor system integration

Digital signal processing and filtering

Serial communication protocols (UART, I2C)

Real-time embedded control on ESP32

Remote monitoring and teleoperation via MQTT

Practical application of the Haversine formula and PID control theory

## Future Enhancements

Multi-sensor fusion (GPS + IMU + magnetometer) for higher robustness

Wheel odometry for dead-reckoning during GPS outages

Computer vision for advanced obstacle classification and path planning

Predictive maintenance using AI models trained on logged sensor CSV data to anticipate component failures
Adaptive path planning with dynamic obstacle mapping

This project demonstrates end-to-end capability in designing, anaysing, troblooshoting, integrating, and controlling a real-world autonomous outdoor robot — from sensor fusion and control algorithms to reliable remote operation
