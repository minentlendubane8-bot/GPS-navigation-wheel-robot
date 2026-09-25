
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

**4. Motor Control**

The steering correction is converted into differential motor speeds:

$$ L=BasePWM-u(t) $$ $$ R=BasePWM+u(t) $$

The resulting commands are sent to the simulated left and right motors, representing the L298N motor-driver stage used in the physical robot.

**5.Feedback Loop**

The feedback loop is used to continuously correct the robot’s heading during navigation. The robot compares the desired waypoint bearing with the measured heading from the QMC5883L compass to determine the heading error.

$$ \alpha = \theta_{waypoint}-\theta_{robot} $$

The calculated error is supplied to the PID controller, which generates a steering correction. This correction is converted into differential speed commands for the left and right motors. As the motors change the robot’s direction, the compass measures the new heading and sends the updated value back to the control system.

This process is repeated continuously.

## Simulation Parameters – Simulink Software

The following parameters were used to configure the Simulink simulation:

Simulation time: 120 s

Time step: 100 ms (0.1 s)

Control update rate: 10 Hz

GPS: Provides the robot’s latitude and longitude

QMC5883L: Provides the robot’s heading

Target location: Defined as the navigation setpoint

PID controller: \(K_p = 0.14,\ K_i = 0,\ K_d = 0.10\)

Motor control: Differential left- and right-motor commands

<img width="1536" height="1024" alt="58796" src="https://github.com/user-attachments/assets/4bd22675-5eee-4d3e-a2cf-a1c43183646b" />


**Simulation Results – PID Steering Response**

The Scope was used to evaluate how the PID controller responds when the steering-angle setpoint changes from 0° to 28°.
<img width="1671" height="941" alt="58797" src="https://github.com/user-attachments/assets/07d5c906-11c4-4f5e-ba10-3cdc6f134969" />

**Interpretation**

The setpoint is the desired steering angle of 28°. The PID output rises toward this value after the step input is applied.

**Rise time** = 0.42 s: the response reaches the specified 10–90% range of the final value in 0.42 seconds.

**Overshoot**= 5.8%: the response temporarily rises above the 28° setpoint, reaching approximately 29.58°, before returning toward the target.

**Settling time** = 1.12 s: the response settles within the specified ±2% band around the setpoint.

**Steady-state error** = 0.3°: after the transient response has settled, the remaining difference from the desired steering angle is approximately 0.3°.

## Obstacle Avoidance

Obstacle avoidance was implemented using the HC-SR04 ultrasonic sensor, a servo motor, and the ESP32 programmed through the Arduino IDE. The ultrasonic sensor measures the distance between the robot and an object in its path.

When an obstacle is detected within the defined distance threshold, the robot stops its motors. The servo then rotates the ultrasonic sensor to scan alternative directions. The implemented scanning sequence is:

RIGHT → LEFT → CENTER

The measured distances are used to determine whether a clearer direction is available. After the obstacle has been handled, the robot can resume its navigation process. 

**MQTT Communication**

MQTT was implemented to provide wireless communication between the mobile application and the ESP32.

The ESP32 connects to the Wi-Fi network and communicates with the MQTT client. Commands from the mobile application are received by the ESP32 and used to control the robot.

The communication was first tested using Start and Stop commands. The test verified that the MQTT connection was working correctly before integrating the communication into the main robot system.

<img width="720" height="682" alt="57751" src="https://github.com/user-attachments/assets/e0f81e21-4d3f-4b08-a148-f3038994b6be" />

## Hardware Development and Testing

**Hardware Assembly and Soldering**

The electronic components were assembled according to the circuit design and mounted onto the robot chassis. Wiring was completed between the ESP32, GPS module, QMC5883L compass, HC-SR04 ultrasonic sensor, servo motor, L298N motor driver, motors, buck converter, and battery supply.

Soldering was used to provide secure electrical connections and reduce loose connections during robot movement. Particular attention was given to power and ground connections, sensor communication lines, and motor-driver connections.

The compass wiring was also improved using shielded cable because electromagnetic interference from the motors and nearby electronics affected the compass readings. A 1 µF decoupling capacitor was placed close to the compass power pins to improve signal stability.

**Bench Testing**

Before operating the complete robot, individual hardware components were tested on the bench.

**The testing was performed in stages:**

**ESP32** – verified programming and GPIO operation.

**GPS** – verified latitude and longitude data.

**QMC5883L**– verified I²C communication and heading measurements.
HC-SR04 – verified distance measurements.
**Servo motor**– verified left/right scanning.

**L298N**– verified forward, reverse and differential motor control.

Power system – checked regulated voltage supplies.

The L298N was also tested separately in Proteus before hardware implementation.

**This approach allowed faults to be identified before all components were integrated.**

**Calibration**

Calibration was performed on the sensors before final system testing.

Compass calibration:

The QMC5883L was rotated through different orientations and calibration offsets were obtained. 

<img width="720" height="604" alt="58894" src="https://github.com/user-attachments/assets/138c28ca-25c5-424e-b39a-d7e177eec18c" />



The resulting heading was compared with a mobile compass application to verify the directional readings. 


<img width="712" height="190" alt="58890" src="https://github.com/user-attachments/assets/0e9815e6-c731-4aac-aae8-b2ff7a1c7ca9" />

<img width="647" height="699" alt="59484" src="https://github.com/user-attachments/assets/3b609f44-9fc8-424f-94ef-62ba79e6877c" />


The sensor was positioned away from motors and other sources of magnetic interference.

**GPS calibration/testing:**

The GPS was tested outdoors with a clear view of the sky. Multiple coordinate readings were collected and averaging/filtering was applied to reduce GPS noise and provide more stable position data.

<img width="597" height="1280" alt="21462" src="https://github.com/user-attachments/assets/6ac2642c-6657-424d-9cc6-a6c70386023c" />

<img width="597" height="1280" alt="21497" src="https://github.com/user-attachments/assets/4675e11d-cfdd-48bd-9a1e-99fc710da39b" />

**Ultrasonic testing:**

The HC-SR04 was tested at different distances to verify obstacle detection before being integrated with the servo scanning mechanism.

**Recording Logs**

During testing, sensor and control data were recorded through the ESP32/Arduino IDE serial monitor.

Typical recorded parameters included:

GPS latitude
GPS longitude
Compass heading
Target bearing
Heading error
Ultrasonic distance
Obstacle status
Motor commands
MQTT commands/status

These logs were useful for comparing the expected algorithm behaviour with the actual hardware response.

**Field Testing**

After successful bench and chassis testing, the robot was tested outdoors.

Outdoor testing was necessary because the GPS requires an open view of the sky for reliable positioning. During field testing, the robot was commanded to navigate toward predefined GPS waypoints while the compass provided heading feedback and the ultrasonic sensor monitored for obstacles.

**The field tests were used to evaluate:**

GPS position stability

Waypoint navigation

Heading correction

PID response

Obstacle detection

Motor behaviour

MQTT communication

Overall autonomous operation

The testing also identified practical limitations such as GPS drift/latency, L298N voltage drop and heating, and the limited detection area of the single ultrasonic sensor.

**Commissioning**

Commissioning was the final stage where the complete robot was checked as an operational system.




