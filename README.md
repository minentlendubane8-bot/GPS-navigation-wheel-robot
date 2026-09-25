
### GPS Autonomous Wheel Robot with Real-Time Obstacle Avoidance

Demo video

https://github.com/user-attachments/assets/d3560fc7-a7fb-413b-817d-3e97b78c01ed

https://github.com/user-attachments/assets/2ce6b894-b9e5-4664-9ef6-b7a479e5acae

# GPS Navigation Autonomous Robot 🤖

An autonomous mobile robot developed as a Bachelor of Engineering Technology in Electrical Engineering final-year project.

The robot uses an **ESP32** as the main controller and integrates GPS navigation, compass heading, ultrasonic obstacle detection, motor control, servo scanning and wireless communication.

---

## 1. Project Overview

The aim of this project was to develop a robot that can navigate using GPS coordinates, determine its direction using a compass, detect obstacles and control its motors automatically.

The main controller is an **ESP32**.

The project was developed through:

- Hardware design
- Circuit design
- Embedded programming
- Sensor integration
- Navigation calculations
- Motor control
- Sensor calibration
- Testing and debugging
- Outdoor field testing

## Box diagram
<img width="698" height="445" alt="57471" src="https://github.com/user-attachments/assets/898e562b-9840-4046-a998-a72c5d14df6c" />

## Circuit diagram 
<img width="720" height="735" alt="57469" src="https://github.com/user-attachments/assets/ffad8537-d310-4db9-9221-8947dd7d3b35" />


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

MQTT protocol enables remote monitoring and control from a  mobile device.

## Hardware Stack

ESP32 · GPS Neo-6M · QMC5883L Magnetometer · Ultrasonic Sensors · Motor Drivers · MQTT over Wi-Fi
Key Technical Achievements

Sub-3 m arrival accuracy under outdoor conditions

Smooth closed-loop steering via PID

Real-time obstacle detection and reaction

Full remote telemetry and control through MQTT

## simulation and results 
MATLAB & Simulink – system modelling and simulation,
<img width="1536" height="1024" alt="58796" src="https://github.com/user-attachments/assets/4bd22675-5eee-4d3e-a2cf-a1c43183646b" />

Results
<img width="1671" height="941" alt="58797" src="https://github.com/user-attachments/assets/07d5c906-11c4-4f5e-ba10-3cdc6f134969" />
Arduino IDE – C/C++ embedded programming.
<img width="720" height="682" alt="57751" src="https://github.com/user-attachments/assets/e0f81e21-4d3f-4b08-a148-f3038994b6be" />

## Skills & Competencies Demonstrated

Multi-sensor system integration

Digital signal processing and filtering

Serial communication protocols (UART, I2C)

Real-time embedded control on ESP32

Remote monitoring and teleoperation via MQTT

Practical application of the Haversine formula and PID control 

## Future Enhancements

Multi-sensor fusion (GPS + IMU + magnetometer) for higher robustness

Wheel odometry for dead-reckoning during GPS outages

Computer vision for advanced obstacle classification and path planning

Predictive maintenance using AI models trained on logged sensor CSV data to anticipate component failures
Adaptive path planning with dynamic obstacle mapping

This project demonstrates end-to-end capability in designing, anaysing, troblooshoting, integrating, and controlling a real-world autonomous outdoor robot — from sensor  and control algorithms to reliable remote operation







