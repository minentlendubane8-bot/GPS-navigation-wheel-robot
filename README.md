# Autonomous GPS Navigation Wheel Robot

## Project Overview

This project is an autonomous GPS navigation wheel robot developed using an ESP32 microcontroller. The robot is designed to navigate between predefined GPS waypoints while detecting and avoiding obstacles.

The project combines GPS positioning, digital compass heading, ultrasonic distance measurement, PID control, motor control, Wi-Fi, and MQTT communication into one embedded system. The main purpose was to develop a system that can collect information from different sensors, process the information using a microcontroller, make navigation decisions, control the motors, and communicate remotely.

## System Description

The robot uses a NEO-6M GPS module to determine its current position. The GPS provides latitude and longitude information, which the ESP32 processes to calculate the distance and direction to the selected waypoint.

A QMC5883L digital compass is used to determine the robot's current heading. The compass communicates with the ESP32 through the I2C protocol. The current heading is compared with the required GPS bearing to determine how much the robot needs to correct its direction.

The robot uses a PID controller to improve its steering. The controller continuously calculates the difference between the desired direction and the current heading. The PID calculation then determines the required steering correction by adjusting the speed of the left and right motors. This allows the robot to make smoother and more controlled movements toward the waypoint.

## GPS Navigation

The robot navigates using predefined GPS waypoints. The ESP32 continuously reads the GPS position and calculates the distance between the robot and the target location.

The system also calculates the bearing to determine the direction of the target. When the robot reaches approximately 3 metres from the waypoint, the waypoint is considered reached and the system can continue to the next waypoint.

## PID Control

PID control is used to improve the robot's heading correction. The proportional part responds to the current heading error, the integral part considers the accumulated error over time, and the derivative part responds to changes in the error.

The combined PID output is used to control the difference between the left and right motor speeds. This allows the robot to continuously correct its direction while moving instead of relying only on fixed turning commands.

## Obstacle Detection and Avoidance

An HC-SR04 ultrasonic sensor is used to detect obstacles in front of the robot. The sensor measures the distance between the robot and nearby objects.

To reduce unstable distance readings, the robot takes multiple measurements and uses median filtering to obtain a more reliable distance value.

The ultrasonic sensor is mounted on a servo motor, allowing it to scan different directions. When an obstacle is detected, the robot stops normal navigation and checks the surrounding area. It then selects a clearer direction and continues its navigation.

## Motor Control

The robot uses two DC motors controlled independently through a motor driver. The ESP32 controls the speed and direction of each motor using PWM and digital control signals.

The PID controller changes the motor speeds according to the required steering correction. By controlling the motors independently, the robot can turn gradually while continuing to move toward the GPS waypoint.

## Remote Monitoring and MQTT

The ESP32 connects to a Wi-Fi network and uses MQTT for remote communication. MQTT allows the robot to receive commands and send information about its current status.

The system can receive commands to start and stop the robot and select different GPS waypoints. The robot can also publish its navigation status for remote monitoring.

This part of the project demonstrates how an embedded device can communicate with a remote system over a wireless network.

## Communication

Different communication methods are used for different parts of the system. The GPS module communicates with the ESP32 through UART, while the digital compass uses I2C. GPIO connections are used for the ultrasonic sensor and motor driver, while PWM is used for motor speed and servo control.

Wi-Fi provides the network connection, and MQTT provides the communication between the robot and the remote monitoring system.

## Software

The project was developed using C++ in the Arduino development environment for the ESP32.

The main libraries used are TinyGPSPlus for GPS data processing, QMC5883LCompass for the digital compass, NewPing for ultrasonic distance measurement, ESP32Servo for servo control, PubSubClient for MQTT communication, HardwareSerial for GPS communication, and Wire for I2C communication.

## Embedded System Integration

The main engineering challenge of the project was integrating several hardware and software components so that they could operate together as one system.

The ESP32 receives information from the sensors, processes the data, calculates the required navigation response, controls the motors, monitors obstacles, and communicates with a remote system. This required understanding sensor communication, data processing, motor control, wireless communication, and software logic.

## Engineering Skills Demonstrated

This project demonstrates practical knowledge of embedded systems, ESP32 development, sensor integration, GPS navigation, I2C and UART communication, PWM control, PID control, motor control, ultrasonic sensing, digital filtering, Wi-Fi, MQTT, remote monitoring, troubleshooting, and system integration.

## Future Improvements

The robot can be improved by introducing a structured state machine to manage navigation, obstacle avoidance, and communication more efficiently.

AI can also be explored for more advanced navigation and control. Sensor and robot data can be saved in CSV files and processed using Python to analyse the robot's behaviour and develop more intelligent control methods.

FreeRTOS tasks could also be introduced to allow different activities, such as GPS processing, MQTT communication, sensor monitoring, and motor control, to operate more independently.

Additional improvements could include camera-based navigation, more advanced sensor fusion, improved path planning, and additional sensors for more reliable navigation.

## Conclusion

This project provided practical experience in designing and integrating an embedded system using multiple sensors, communication protocols, control methods, and motor systems.

The robot demonstrates how sensor information can be collected and processed by a microcontroller to make real-time decisions, control physical hardware, and communicate with a remote system. It also provided practical experience in PID control, sensor filtering, troubleshooting, wireless communication, and embedded system development.

## Project Demonstration
A short video demonstration of the robot is available on Google Drive and shows the robot navigating using GPS, PID control, and obstacle detection.
