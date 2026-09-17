// ================================================================
// AUTONOMOUS GPS NAVIGATION WHEEL ROBOT
// ESP32 + GPS + QMC5883L + Ultrasonic + Servo + MQTT
// CLOSED-LOOP PID HEADING CONTROL
// ================================================================

#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <QMC5883LCompass.h>
#include <HardwareSerial.h>
#include <NewPing.h>
#include <ESP32Servo.h>

// ================================================================
// MQTT CONFIGURATION
// ================================================================

#define MQTT_SERVER   "broker.hivemq.com"
#define MQTT_PORT     1883
#define MQTT_CLIENTID "ESP32_Waypoint_Robot_001"

// Wi-Fi
const char* WIFI_SSID = "YOUR_WIFI_NAME";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";

// MQTT topics
const char* TOPIC_CMD_START = "mine/start/robot";
const char* TOPIC_CMD_STOP  = "mine/stop/robot";

const char* TOPIC_WP_A = "mine/waypoint/a";
const char* TOPIC_WP_B = "mine/waypoint/b";
const char* TOPIC_WP_C = "mine/waypoint/c";

const char* TOPIC_STATUS = "mine/status";

// Wi-Fi and MQTT objects
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// Robot control flags
volatile bool runEnabled = false;
volatile int targetWP = -1;


// ================================================================
// GPS WAYPOINTS
// ================================================================

struct Waypoint
{
  double lat;
  double lon;
  const char* name;
};

static const Waypoint WPS[] =
{
  { -34.00097229769423, 25.671539797928926, "A" },
  { -34.00099342214387, 25.671622946404103, "B" },
  { -34.00101677021368, 25.671580031062078, "C" }
};

static const int WPS_COUNT = sizeof(WPS) / sizeof(WPS[0]);

// Robot considers waypoint reached inside this radius
const double ARRIVE_M = 3.0;


// ================================================================
// COMPASS CALIBRATION
// ================================================================

const float OffsX = 314.0f;
const float OffsY = 47.5f;

const float ScaleX = 0.68695f;
const float ScaleY = 0.75273f;

// Magnetic declination
const float DECLINATION = -30.30f;

QMC5883LCompass compass;


// ================================================================
// MOTOR PINS
// ================================================================

const int ENA = 4;
const int IN1 = 16;
const int IN2 = 17;

const int ENB = 27;
const int IN3 = 15;
const int IN4 = 13;


// ================================================================
// GPS
// ================================================================

HardwareSerial gpsSerial(1);

TinyGPSPlus gps;

const int GPS_RX = 14;
const int GPS_TX = 26;
const int GPS_BAUD = 9600;


// ================================================================
// ULTRASONIC SENSOR
// ================================================================

const uint8_t US_TRIG = 25;
const uint8_t US_ECHO = 34;

const uint16_t US_MAX_DISTANCE_CM = 400;

// Distance at which obstacle avoidance starts
const uint16_t US_BLOCK_CM = 20;

// Distance considered clear
const uint16_t US_CLEAR_CM = 30;

NewPing sonar(
  US_TRIG,
  US_ECHO,
  US_MAX_DISTANCE_CM
);


// ================================================================
// SERVO
// ================================================================

Servo pan;

const int SERVO_PIN = 18;

const int SERVO_LEFT   = 160;
const int SERVO_CENTER = 90;
const int SERVO_RIGHT  = 20;


// ================================================================
// PID CONTROLLER
// ================================================================
//
// PID uses:
//
// P = Proportional
// I = Integral
// D = Derivative
//
// Error = desired heading - actual heading
//
// PID output determines how much the left/right
// motor speeds should be different.
//
// Positive output  -> one direction
// Negative output  -> opposite direction
//
// These values are starting values.
// They should be tuned experimentally on the robot.
//

float Kp = 2.0f;
float Ki = 0.02f;
float Kd = 0.8f;

// PID variables
float pidIntegral = 0.0f;
float previousError = 0.0f;

unsigned long previousPIDTime = 0;

// Integral protection
const float INTEGRAL_LIMIT = 100.0f;

// Maximum PID steering correction
const float PID_OUTPUT_LIMIT = 100.0f;


// ================================================================
// MOTOR SPEED SETTINGS
// ================================================================

// Base forward speed
const int BASE_SPEED = 130;

// Minimum motor speed
const int MIN_SPEED = 0;

// Maximum PWM
const int MAX_PWM = 255;


// ================================================================
// MATH FUNCTIONS
// ================================================================

double deg2rad(double d)
{
  return d * PI / 180.0;
}


double rad2deg(double r)
{
  return r * 180.0 / PI;
}


// ---------------------------------------------------------------
// Keep angle between -180° and +180°
// ---------------------------------------------------------------

float wrap180(float angle)
{
  while (angle > 180.0f)
    angle -= 360.0f;

  while (angle < -180.0f)
    angle += 360.0f;

  return angle;
}


// ================================================================
// HAVERSINE DISTANCE
// ================================================================
//
// Calculates distance between two GPS coordinates.
//
// The result is returned in metres.
//
// This is useful because GPS coordinates are given
// as latitude and longitude rather than X/Y metres.
// ================================================================

double haversine_m(
  double lat1,
  double lon1,
  double lat2,
  double lon2)
{
  const double R = 6371000.0;

  double p1 = deg2rad(lat1);
  double p2 = deg2rad(lat2);

  double dp = p2 - p1;
  double dl = deg2rad(lon2 - lon1);

  double a =
    sin(dp / 2) * sin(dp / 2) +
    cos(p1) * cos(p2) *
    sin(dl / 2) * sin(dl / 2);

  double c = 2.0 * atan2(
    sqrt(a),
    sqrt(1.0 - a)
  );

  return R * c;
}


// ================================================================
// BEARING CALCULATION
// ================================================================
//
// Calculates the direction from the current GPS position
// to the target GPS waypoint.
//
// Result:
// 0°   = North
// 90°  = East
// 180° = South
// 270° = West
// ================================================================

double bearing_deg(
  double lat1,
  double lon1,
  double lat2,
  double lon2)
{
  double p1 = deg2rad(lat1);
  double p2 = deg2rad(lat2);

  double dL = deg2rad(lon2 - lon1);

  double y = sin(dL) * cos(p2);

  double x =
    cos(p1) * sin(p2) -
    sin(p1) * cos(p2) * cos(dL);

  double bearing =
    rad2deg(atan2(y, x));

  if (bearing < 0)
    bearing += 360.0;

  return bearing;
}


// ================================================================
// READ CALIBRATED TRUE HEADING
// ================================================================

float readHeadingTrueDeg()
{
  // Read magnetometer
  compass.read();

  // Get raw measurements
  int16_t rx = compass.getX();
  int16_t ry = compass.getY();

  // Apply calibration
  float x =
    ((float)rx - OffsX) * ScaleX;

  float y =
    ((float)ry - OffsY) * ScaleY;

  // Calculate magnetic heading
  float magRad = atan2f(y, x);

  if (magRad < 0)
    magRad += 2 * PI;

  float magDeg =
    rad2deg(magRad);

  // Apply magnetic declination
  float trueHeading =
    fmod(
      magDeg + DECLINATION + 360.0f,
      360.0f
    );

  return trueHeading;
}


// ================================================================
// ULTRASONIC DISTANCE
// ================================================================
//
// Takes three readings and uses the median result.
// This helps reduce unstable ultrasonic measurements.
// ================================================================

uint16_t ultrasonicReadCM()
{
  unsigned int uS =
    sonar.ping_median(3);

  if (uS == 0)
    return US_MAX_DISTANCE_CM + 1;

  return uS / US_ROUNDTRIP_CM;
}


// ================================================================
// MOTOR CONTROL FUNCTIONS
// ================================================================

// Stop both motors
void motorsStop()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}


// ---------------------------------------------------------------
// Set left and right motor speeds
// ---------------------------------------------------------------
//
// Positive speed = forward
// Negative speed = backward
//
// This function allows differential steering.
// ================================================================

void setMotorSpeeds(int leftSpeed, int rightSpeed)
{
  // Limit speeds
  leftSpeed =
    constrain(leftSpeed, -MAX_PWM, MAX_PWM);

  rightSpeed =
    constrain(rightSpeed, -MAX_PWM, MAX_PWM);


  // ============================
  // LEFT MOTOR
  // ============================

  if (leftSpeed > 0)
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);

    analogWrite(
      ENA,
      leftSpeed
    );
  }
  else if (leftSpeed < 0)
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);

    analogWrite(
      ENA,
      abs(leftSpeed)
    );
  }
  else
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);

    analogWrite(ENA, 0);
  }


  // ============================
  // RIGHT MOTOR
  // ============================

  if (rightSpeed > 0)
  {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);

    analogWrite(
      ENB,
      rightSpeed
    );
  }
  else if (rightSpeed < 0)
  {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

    analogWrite(
      ENB,
      abs(rightSpeed)
    );
  }
  else
  {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);

    analogWrite(ENB, 0);
  }
}


// ================================================================
// PID RESET
// ================================================================
//
// Called when starting a new navigation operation.
// ================================================================

void resetPID()
{
  pidIntegral = 0.0f;
  previousError = 0.0f;

  previousPIDTime =
    millis();
}


// ================================================================
// PID HEADING CONTROL
// ================================================================
//
// Inputs:
// targetBearing = desired direction
// currentHeading = actual compass heading
//
// Output:
// Motor speed correction
// ================================================================

float calculatePID(
  float targetBearing,
  float currentHeading)
{
  // Calculate heading error
  float error =
    wrap180(
      targetBearing -
      currentHeading
    );


  // Calculate elapsed time
  unsigned long now =
    millis();

  float dt =
    (now - previousPIDTime) / 1000.0f;

  // Protect against zero/very small dt
  if (dt <= 0.001f)
    dt = 0.001f;

  previousPIDTime = now;


  // ============================================================
  // PROPORTIONAL TERM
  // ============================================================
  //
  // Larger error produces larger correction.
  //

  float P =
    Kp * error;


  // ============================================================
  // INTEGRAL TERM
  // ============================================================
  //
  // Accumulates previous error.
  //
  // Useful for reducing small persistent steering errors.
  //

  pidIntegral +=
    error * dt;

  // Prevent integral wind-up
  pidIntegral =
    constrain(
      pidIntegral,
      -INTEGRAL_LIMIT,
      INTEGRAL_LIMIT
    );

  float I =
    Ki * pidIntegral;


  // ============================================================
  // DERIVATIVE TERM
  // ============================================================
  //
  // Measures how quickly the error is changing.
  //

  float derivative =
    (error - previousError) / dt;

  float D =
    Kd * derivative;

  previousError =
    error;


  // ============================================================
  // TOTAL PID OUTPUT
  // ============================================================

  float output =
    P + I + D;

  output =
    constrain(
      output,
      -PID_OUTPUT_LIMIT,
      PID_OUTPUT_LIMIT
    );


  // Debug information
  Serial.printf(
    "[PID] Error=%.2f P=%.2f I=%.2f D=%.2f Output=%.2f\n",
    error,
    P,
    I,
    D,
    output
  );

  return output;
}


// ================================================================
// PID NAVIGATION MOTOR CONTROL
// ================================================================
//
// PID output is used as differential steering.
//
// Example:
//
// Output = +40
//
// Left motor  = 130 - 40 = 90
// Right motor = 130 + 40 = 170
//
// The difference in speed causes the robot to turn.
//
// ================================================================

void driveWithPID(
  float targetBearing,
  float currentHeading)
{
  float pidOutput =
    calculatePID(
      targetBearing,
      currentHeading
    );


  // Calculate motor speeds
  int leftMotor =
    BASE_SPEED - pidOutput;

  int rightMotor =
    BASE_SPEED + pidOutput;


  // Limit motor speeds
  leftMotor =
    constrain(
      leftMotor,
      MIN_SPEED,
      MAX_PWM
    );

  rightMotor =
    constrain(
      rightMotor,
      MIN_SPEED,
      MAX_PWM
    );


  // Apply motor speeds
  setMotorSpeeds(
    leftMotor,
    rightMotor
  );


  Serial.printf(
    "[MOTOR] Left=%d Right=%d\n",
    leftMotor,
    rightMotor
  );
}


// ================================================================
// MQTT CALLBACK
// ================================================================

void mqttCallback(
  char* topic,
  byte* message,
  unsigned int length)
{
  String msg;

  for (
    unsigned int i = 0;
    i < length;
    i++
  )
  {
    msg +=
      (char)message[i];
  }

  msg.trim();
  msg.toUpperCase();


  // ============================================================
  // START COMMAND
  // ============================================================

  if (
    String(topic) == TOPIC_CMD_START &&
    (
      msg == "1" ||
      msg == "START" ||
      msg == "ON"
    )
  )
  {
    runEnabled = true;

    resetPID();

    mqttClient.publish(
      TOPIC_STATUS,
      "ROBOT STARTED"
    );
  }


  // ============================================================
  // STOP COMMAND
  // ============================================================

  else if (
    String(topic) == TOPIC_CMD_STOP &&
    (
      msg == "1" ||
      msg == "STOP" ||
      msg == "OFF"
    )
  )
  {
    runEnabled = false;

    motorsStop();

    resetPID();

    mqttClient.publish(
      TOPIC_STATUS,
      "ROBOT STOPPED"
    );
  }


  // ============================================================
  // WAYPOINT A
  // ============================================================

  else if (
    String(topic) == TOPIC_WP_A
  )
  {
    targetWP = 0;

    resetPID();

    mqttClient.publish(
      TOPIC_STATUS,
      "WAYPOINT A SELECTED"
    );
  }


  // ============================================================
  // WAYPOINT B
  // ============================================================

  else if (
    String(topic) == TOPIC_WP_B
  )
  {
    targetWP = 1;

    resetPID();

    mqttClient.publish(
      TOPIC_STATUS,
      "WAYPOINT B SELECTED"
    );
  }


  // ============================================================
  // WAYPOINT C
  // ============================================================

  else if (
    String(topic) == TOPIC_WP_C
  )
  {
    targetWP = 2;

    resetPID();

    mqttClient.publish(
      TOPIC_STATUS,
      "WAYPOINT C SELECTED"
    );
  }
}


// ================================================================
// WIFI CONNECTION
// ================================================================

void connectToWiFi()
{
  if (
    WiFi.status() ==
    WL_CONNECTED
  )
    return;

  WiFi.mode(WIFI_STA);

  WiFi.begin(
    WIFI_SSID,
    WIFI_PASS
  );

  Serial.print(
    "Connecting WiFi"
  );

  while (
    WiFi.status() !=
    WL_CONNECTED
  )
  {
    delay(500);

    Serial.print(".");
  }

  Serial.println(
    "\nWiFi Connected!"
  );
}


// ================================================================
// MQTT CONNECTION
// ================================================================

void reconnectMQTT()
{
  while (
    !mqttClient.connected()
  )
  {
    Serial.println(
      "Connecting MQTT..."
    );

    if (
      mqttClient.connect(
        MQTT_CLIENTID
      )
    )
    {
      Serial.println(
        "MQTT Connected!"
      );

      mqttClient.subscribe(
        TOPIC_CMD_START
      );

      mqttClient.subscribe(
        TOPIC_CMD_STOP
      );

      mqttClient.subscribe(
        TOPIC_WP_A
      );

      mqttClient.subscribe(
        TOPIC_WP_B
      );

      mqttClient.subscribe(
        TOPIC_WP_C
      );
    }
    else
    {
      delay(2000);
    }
  }
}


// ================================================================
// SERVO SCANNING
// ================================================================

uint16_t scanDirection(
  bool left)
{
  uint16_t minDist =
    999;


  // ============================================================
  // LEFT SCAN
  // ============================================================

  if (left)
  {
    for (
      int pos = SERVO_CENTER;
      pos <= SERVO_LEFT;
      pos += 15
    )
    {
      pan.write(pos);

      delay(80);

      uint16_t d =
        ultrasonicReadCM();

      if (
        d < minDist
      )
      {
        minDist = d;
      }
    }
  }


  // ============================================================
  // RIGHT SCAN
  // ============================================================

  else
  {
    for (
      int pos = SERVO_CENTER;
      pos >= SERVO_RIGHT;
      pos -= 15
    )
    {
      pan.write(pos);

      delay(80);

      uint16_t d =
        ultrasonicReadCM();

      if (
        d < minDist
      )
      {
        minDist = d;
      }
    }
  }


  // Return sensor to centre
  pan.write(
    SERVO_CENTER
  );

  return minDist;
}


// ================================================================
// OBSTACLE AVOIDANCE
// ================================================================

void avoidObstacle()
{
  mqttClient.publish(
    TOPIC_STATUS,
    "OBSTACLE DETECTED"
  );

  Serial.println(
    "[EVENT] Obstacle detected"
  );


  // Stop robot
  motorsStop();

  int attempts = 0;


  while (true)
  {
    // Keep MQTT communication active
    mqttClient.loop();


    // Scan left
    uint16_t leftClear =
      scanDirection(true);


    // Scan right
    uint16_t rightClear =
      scanDirection(false);


    Serial.printf(
      "[AVOID] Left=%ucm Right=%ucm Attempt=%d\n",
      leftClear,
      rightClear,
      attempts + 1
    );


    // ==========================================================
    // BOTH DIRECTIONS BLOCKED
    // ==========================================================

    if (
      leftClear <= US_BLOCK_CM &&
      rightClear <= US_BLOCK_CM
    )
    {
      Serial.println(
        "[AVOID] Both sides blocked"
      );

      // Reverse
      setMotorSpeeds(
        -120,
        -120
      );

      delay(700);

      motorsStop();

      delay(200);

      attempts++;

      continue;
    }


    // ==========================================================
    // SELECT CLEARER DIRECTION
    // ==========================================================

    mqttClient.publish(
      TOPIC_STATUS,
      "AVOIDING"
    );


    if (
      leftClear > rightClear
    )
    {
      Serial.println(
        "[AVOID] Turning LEFT"
      );

      // Turn left
      setMotorSpeeds(
        0,
        130
      );
    }
    else
    {
      Serial.println(
        "[AVOID] Turning RIGHT"
      );

      // Turn right
      setMotorSpeeds(
        130,
        0
      );
    }


    // Allow robot to turn
    delay(650);

    motorsStop();

    delay(150);


    // Move forward slightly
    setMotorSpeeds(
      150,
      150
    );

    delay(500);

    motorsStop();

    delay(200);


    // Check path again
    uint16_t postNudge =
      ultrasonicReadCM();


    if (
      postNudge >
      US_CLEAR_CM
    )
    {
      Serial.println(
        "[AVOID] Path clear"
      );

      break;
    }


    // Still blocked
    Serial.println(
      "[AVOID] Still blocked"
    );


    setMotorSpeeds(
      -120,
      -120
    );

    delay(800);

    motorsStop();

    delay(200);

    attempts++;
  }


  pan.write(
    SERVO_CENTER
  );

  delay(150);


  // Reset PID after obstacle avoidance
  resetPID();
}


// ================================================================
// SETUP
// ================================================================

void setup()
{
  Serial.begin(
    115200
  );


  // ============================================================
  // I2C
  // ============================================================

  Wire.begin(
    21,
    22
  );


  // ============================================================
  // WIFI
  // ============================================================

  connectToWiFi();


  // ============================================================
  // MQTT
  // ============================================================

  mqttClient.setServer(
    MQTT_SERVER,
    MQTT_PORT
  );

  mqttClient.setCallback(
    mqttCallback
  );

  reconnectMQTT();


  // ============================================================
  // COMPASS
  // ============================================================

  compass.init();


  // ============================================================
  // GPS UART
  // ============================================================

  gpsSerial.begin(
    GPS_BAUD,
    SERIAL_8N1,
    GPS_RX,
    GPS_TX
  );


  // ============================================================
  // MOTOR PINS
  // ============================================================

  pinMode(
    IN1,
    OUTPUT
  );

  pinMode(
    IN2,
    OUTPUT
  );

  pinMode(
    IN3,
    OUTPUT
  );

  pinMode(
    IN4,
    OUTPUT
  );

  pinMode(
    ENA,
    OUTPUT
  );

  pinMode(
    ENB,
    OUTPUT
  );


  // ============================================================
  // SERVO
  // ============================================================

  pan.attach(
    SERVO_PIN
  );

  pan.write(
    SERVO_CENTER
  );


  // Stop motors
  motorsStop();


  // Initialise PID timer
  resetPID();


  Serial.println(
    "================================="
  );

  Serial.println(
    "GPS ROBOT SYSTEM READY"
  );

  Serial.println(
    "CLOSED-LOOP PID NAVIGATION"
  );

  Serial.println(
    "================================="
  );
}


// ================================================================
// MAIN LOOP
// ================================================================

void loop()
{
  // ============================================================
  // MAINTAIN WIFI
  // ============================================================

  if (
    WiFi.status() !=
    WL_CONNECTED
  )
  {
    connectToWiFi();
  }


  // ============================================================
  // MAINTAIN MQTT
  // ============================================================

  if (
    !mqttClient.connected()
  )
  {
    reconnectMQTT();
  }

  mqttClient.loop();


  // ============================================================
  // ROBOT NOT ENABLED
  // ============================================================

  if (
    !runEnabled ||
    targetWP == -1
  )
  {
    motorsStop();

    delay(50);

    return;
  }


  // ============================================================
  // READ GPS DATA
  // ============================================================

  while (
    gpsSerial.available()
  )
  {
    gps.encode(
      gpsSerial.read()
    );
  }


  // Wait for valid GPS position
  if (
    !gps.location.isValid()
  )
  {
    motorsStop();

    Serial.println(
      "Waiting for GPS fix..."
    );

    delay(500);

    return;
  }


  // ============================================================
  // GET CURRENT POSITION
  // ============================================================

  double lat =
    gps.location.lat();

  double lon =
    gps.location.lng();


  // ============================================================
  // CALCULATE DISTANCE TO TARGET
  // ============================================================

  double dist =
    haversine_m(
      lat,
      lon,
      WPS[targetWP].lat,
      WPS[targetWP].lon
    );


  // ============================================================
  // CALCULATE TARGET BEARING
  // ============================================================

  double bearing =
    bearing_deg(
      lat,
      lon,
      WPS[targetWP].lat,
      WPS[targetWP].lon
    );


  // ============================================================
  // READ CURRENT COMPASS HEADING
  // ============================================================

  float heading =
    readHeadingTrueDeg();


  // ============================================================
  // CALCULATE HEADING ERROR
  // ============================================================

  float error =
    wrap180(
      bearing - heading
    );


  // ============================================================
  // READ FRONT OBSTACLE DISTANCE
  // ============================================================

  uint16_t frontDist =
    ultrasonicReadCM();


  // ============================================================
  // SERIAL NAVIGATION INFORMATION
  // ============================================================

  Serial.printf(
    "[NAV] Target=%s "
    "Dist=%.2fm "
    "Bearing=%.1f° "
    "Heading=%.1f° "
    "Error=%.1f° "
    "Front=%ucm\n",

    WPS[targetWP].name,
    dist,
    bearing,
    heading,
    error,
    frontDist
  );


  // ============================================================
  // OBSTACLE DETECTION
  // ============================================================

  if (
    frontDist <
    US_BLOCK_CM
  )
  {
    avoidObstacle();

    return;
  }


  // ============================================================
  // WAYPOINT REACHED
  // ============================================================

  if (
    dist <
    ARRIVE_M
  )
  {
    motorsStop();


    char msg[50];

    sprintf(
      msg,
      "REACHED WAYPOINT %s",
      WPS[targetWP].name
    );


    mqttClient.publish(
      TOPIC_STATUS,
      msg
    );


    Serial.printf(
      "[NAV] Reached waypoint %s\n",
      WPS[targetWP].name
    );


    // Clear target
    targetWP = -1;


    // Reset controller
    resetPID();


    return;
  }


  // ============================================================
  // PID NAVIGATION
  // ============================================================
  //
  // Instead of:
  //
  // Error > 10°  -> fixed left turn
  // Error < -10° -> fixed right turn
  //
  // the PID controller continuously calculates
  // a steering correction.
  //
  // ============================================================

  driveWithPID(
    bearing,
    heading
  );


  // Small control-loop interval
  delay(50);
}
 
