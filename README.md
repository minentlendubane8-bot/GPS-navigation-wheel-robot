
### GPS Autonomous Wheel Robot with Real-Time Obstacle Avoidance

An autonomous mobile robot developed as a Bachelor of Engineering Technology in Electrical Engineering final-year project.

The robot uses an **ESP32** as the main controller and integrates GPS navigation, compass heading, ultrasonic obstacle detection, motor control, servo scanning and wireless communication.

Demo video

https://github.com/user-attachments/assets/d3560fc7-a7fb-413b-817d-3e97b78c01ed

https://github.com/user-attachments/assets/2ce6b894-b9e5-4664-9ef6-b7a479e5acae


## Project Overview

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

## System Block Diagram

The following block diagram shows how the main inputs and outputs are connected to the ESP32.

<img width="698" height="445" alt="57471" src="https://github.com/user-attachments/assets/898e562b-9840-4046-a998-a72c5d14df6c" />

**Inputs**

The ESP32 receives information from:

- **GPS Module** – provides position information.
- **Compass Sensor** – provides heading information.
- **Ultrasonic Sensor** – measures distance to obstacles.

**Outputs**
The ESP32 controls:

- **L298N Motor Driver**
- **Motor 1**
- **Motor 2**
- **Servo Motor**

The servo motor is used to move the ultrasonic sensor for obstacle scanning.

##  Circuit diagram 
The circuit diagram shows the electrical implementation of the autonomous robot, including the power supply, voltage regulation, communication interfaces, ESP32 pin connections, motor-driver connections and signal-level protection.

<img width="720" height="735" alt="57469" src="https://github.com/user-attachments/assets/ffad8537-d310-4db9-9221-8947dd7d3b35" /> 

**Power Supply**

The robot is powered by a **12 V battery**.

The 12 V supply is used for the motor-drive section through the L298N motor driver. A regulated supply is used for the lower-voltage electronic components(buck convert two output 3.3V and 5V).

The ESP32 operates at **3.3 V**, while other components in the system require different supply voltages.

| Component | Supply Voltage |
|---|---:|
| ESP32 | 3.3 V |
| GPS NEO-6M | 3.3 V |
| QMC5883L Compass | 3.3 V |
| L298N Motor Supply | 12 V |
| L298N Logic Supply | 5 V |
| Servo Motor | 5 V |
| HC-SR04 | 5 V |

**Common Ground**

A common ground is used between the ESP32 and the connected electronic circuits.

This provides a **common voltage reference for the signal connections**, allowing the ESP32 to correctly interpret signals from the GPS, compass, ultrasonic sensor and other connected devices.

The servo and external 3.3 V supply also share ground with the ESP32.

**ESP32 Pin Configuration**

The ESP32 acts as the main interface between the sensors and actuators.

| Component | Signal | ESP32 Pin |
|---|---|---|
| GPS NEO-6M | RX | GPIO 21 |
| GPS NEO-6M | TX | GPIO 23 |
| QMC5883L | SDA | GPIO 21 |
| QMC5883L | SCL | GPIO 22 |
| HC-SR04 | Trigger | ESP32 GPIO |
| HC-SR04 | Echo | ESP32 GPIO through voltage divider |
| Servo | PWM Signal | ESP32 PWM GPIO |
| L298N | IN1–IN4 | ESP32 Digital GPIO |
| L298N | ENA / ENB | ESP32 PWM GPIO |

> The ESP32 uses UART for GPS communication and I²C for the compass interface.

**GPS Communication**

The GPS NEO-6M communicates with the ESP32 using **UART serial communication**.

GPS TX  ─────────► ESP32 RX
GPS RX  ◄───────── ESP32 TX

The GPS continuously sends position information to the ESP32, which processes the received data for navigation.

**Compass Communication**

The QMC5883L compass uses the I²C communication interface.

Compass SDA ─────► ESP32 GPIO 21
Compass SCL ─────► ESP32 GPIO 22

The compass provides heading information used by the navigation and heading-control system.

**HC-SR04 Voltage Divider**

The HC-SR04 ultrasonic sensor operates at 5 V, while the ESP32 uses 3.3 V logic.

The Echo output of the HC-SR04 can therefore produce a 5 V signal, which is higher than the ESP32 GPIO input level.

A two-resistor voltage divider was implemented between the HC-SR04 Echo output and the ESP32 input to reduce the signal voltage.

The voltage divider was designed using E12-series resistor values.

The voltage-divider relationship is:

Vout = Vin × R2 / (R1 + R2)

Using:

Vin = 5 V
R1  = 47 Ω
R2  = 100 Ω

The expected output is approximately:

Vout = 5 × 100 / (47 + 100)

Vout ≈ 3.4 V

The calculated divider current was approximately:

I = 1.7 V / 147 Ω
I ≈ 11.6 mA

This design was selected to provide a voltage close to the required 3.3 V logic level while keeping the divider current below the specified 12 mA limit.

**Why the Voltage Divider Was Required**

The voltage divider provides an interface between the 5 V HC-SR04 Echo signal and the 3.3 V ESP32 GPIO input.

This was an important part of the circuit design because the sensor and microcontroller operate at different logic levels.

## Navigation Theory and Algorithm
The robot must determine its current position and heading and use these measurements to navigate toward a predefined GPS waypoint.

**The navigation problem therefore has three main quantities:**

**Current position**— obtained from the GPS.

**Current heading** — obtained from the QMC5883L compass.

**Target position**— predefined latitude and longitude.

The navigation system then calculates:

Distance from the robot to the target.
Bearing from the robot to the target.
Difference between the required bearing and the current heading.
Steering correction required to reduce this error. 

<img width="720" height="408" alt="58896" src="https://github.com/user-attachments/assets/1c5b5191-58b3-4b50-8084-520e35c3aee6" />

**1. Robot Heading and Waypoint Bearing**

The first important concept is the difference between where the robot is facing and where the target is located.

The compass provides the robot heading:

$$ \theta_{robot} $$

The GPS coordinates of the robot and target are used to calculate the direction toward the waypoint:

$$ \theta_{waypoint} $$

The navigation diagram represents these two directions relative to North
                          
The heading error is calculated as:

$$ \alpha = \theta_{waypoint}-\theta_{robot} $$

For example, the Simulink model shows:

Robot heading = 87.6°
Waypoint bearing = 102.7°

Therefore:

$$ \alpha = 102.7^\circ-87.6^\circ $$ $$ \boxed{\alpha=15.1^\circ} $$

So the robot has a 15.1° heading error and the controller must generate a steering correction.

This is important because the robot does not simply move toward the GPS coordinate. It continuously compares its required direction with its measured direction.

**2. Heading Error Wrapping**

An important software consideration is that angles are circular.

For example, a robot heading of 359° and a target bearing of 1° should produce a small 2° correction—not a 358° correction.

Therefore, the heading error is constrained to:

$$ -180^\circ \leq \alpha < 180^\circ $$

Conceptually:

α = wrap(θwaypoint − θrobot)

This makes the navigation algorithm select the shortest angular correction.
the equation becomes part of the actual navigation algorithm.

**3.PID Control**

Once the heading error has been calculated and wrapped between −180° and +180°, the error becomes the input to the PID controller.

The purpose of the PID controller is to reduce the heading error and produce an appropriate steering correction. Instead of simply commanding the robot to turn left or right, the controller determines how much correction should be applied based on the magnitude and behaviour of the error.

The general PID equation used to describe the controller is:

$$ u(t)=K_p e(t)+K_i\int e(t)\,dt+K_d\frac{de(t)}{dt} $$

where:

e(t) = heading error

K_p = proportional gain

K_i= integral gain

K_d = derivative gain

u(t)= steering/control output


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







