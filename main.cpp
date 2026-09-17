//CLOSE LOOP PID CODE
#include <WiFi.h>                                  // ESP32 WIFI connection
#include <PubSubClient.h>                          //MQTT message
#include <Wire.h>                                  //I2C for compass sensor
#include <TinyGPSPlus.h>                           //Parsing MNEA GPS data
#include <QMC5883LCompass.h>                       //QMC5883L magnetometer
#include <HardwareSerial.h>                        // serial port
#include <NewPing.h>                               //for ultrasonic sensor with better accuracy
#include <ESP32Servo.h>                            // motor control on ESP32
#define MQTT_SERVER   "broker.hivemq.com"          // Free public MQTT broker
#define MQTT_PORT     1883                         //Default MQTT port
#define MQTT_CLIENTID "ESP32_Waypoint_Robot_001"   // Unique ID for the robot


// ================== Wi-Fi + MQTT CONFIG==================
const char* WIFI_SSID = "Galaxy S23 Ultra 94A8";  // mobile hostspot name
const char* WIFI_PASS = "BigDaddy4U";             // hospot password            

//==============MQTT topics for remot control==================
const char* TOPIC_CMD_START = "mine/start/robot";
const char* TOPIC_CMD_STOP  = "mine/stop/robot";
const char* TOPIC_WP_A      = "mine/waypoint/a";
const char* TOPIC_WP_B      = "mine/waypoint/b";
const char* TOPIC_WP_C      = "mine/waypoint/c";
const char* TOPIC_STATUS    = "mine/status";   // status updates published here

WiFiClient espClient;                //WIFi client object
PubSubClient mqttClient(espClient);  // MQTT client uses WiFi client
volatile bool runEnabled = false;    // Flag: true = robot move
volatile int targetWP = -1;          // No waypoint is selected

// ================== Waypoints ==================
struct Waypoint { double lat, lon; const char* name; };
static const Waypoint WPS[] = {
  { -34.00097229769423, 25.671539797928926, "A" },
  { -34.00099342214387, 25.671622946404103, "B" },
  { -34.00101677021368, 25.671580031062078, "C" }
};
static const int WPS_COUNT = sizeof(WPS)/sizeof(WPS[0]);
const double ARRIVE_M = 3.0;

// ================== Compass Calibration ==================
const float OffsX = 314.0f, OffsY = 47.5f;
const float ScaleX = 0.68695f, ScaleY = 0.75273f;
const float DECLINATION = -30.30f;

// ================== Motor Pins ==================
const int ENA = 4;
const int IN1 = 16;
const int IN2 = 17;
const int ENB = 27;
const int IN3 = 15;
const int IN4 = 13;

// ================== GPS ==================
HardwareSerial gpsSerial(1);
TinyGPSPlus gps;
const int GPS_RX = 14;  // GPS TX -> 14
const int GPS_TX = 26;  // GPS RX -> 26
const int GPS_BAUD = 9600; // NEO-6M default baud rate

// ================== Compass ==================
QMC5883LCompass compass;

// ================== Ultrasonic ==================
const uint8_t US_TRIG = 25;
const uint8_t US_ECHO = 34;
const uint16_t US_MAX_DISTANCE_CM = 400;
const uint16_t US_BLOCK_CM = 20;
const uint16_t US_CLEAR_CM = 30;
NewPing sonar(US_TRIG, US_ECHO, US_MAX_DISTANCE_CM);

// ================== Servo ==================
Servo pan;
const int SERVO_PIN = 18;
const int SERVO_LEFT = 160;
const int SERVO_CENTER = 90;
const int SERVO_RIGHT = 20;

// ================== Math Helpers ==================
double deg2rad(double d){ return d * PI / 180.0; }
double rad2deg(double r){ return r * 180.0 / PI; }

// Wrap angle to -180 to 180 degree for PID  erro calculation
float wrap180(float a){ while(a>180)a-=360; while(a<-180)a+=360; return a; }

//Harversin formula- Calculate distance between 2 GPS points in meters

double haversine_m(double lat1,double lon1,double lat2,double lon2){
  const double R=6371000.0; // Earth  radius in meters
  double p1=deg2rad(lat1),p2=deg2rad(lat2);
  double dp=p2-p1,dl=deg2rad(lon2-lon1);
  double a=sin(dp/2)*sin(dp/2)+cos(p1)*cos(p2)*sin(dl/2)*sin(dl/2);
  return R*2*atan2(sqrt(a),sqrt(1-a));
}

//Calculate bearing angle from current position to target

double bearing_deg(double lat1,double lon1,double lat2,double lon2){
  double p1=deg2rad(lat1), p2=deg2rad(lat2), dL=deg2rad(lon2-lon1);
  double y=sin(dL)*cos(p2);
  double x=cos(p1)*sin(p2)-sin(p1)*cos(p2)*cos(dL);
  double br=rad2deg(atan2(y,x));
  if(br < 0) br += 360;
  return br;
} 

// Reading true heading - applies calibration  and declination

float readHeadingTrueDeg(){
  compass.read();  // Read raw magnetometer
  int16_t rx=compass.getX(), ry=compass.getY();
  float x=((float)rx-OffsX)*ScaleX;
  float y=((float)ry-OffsY)*ScaleY;
  float magRad=atan2f(y,x);
  if(magRad<0) magRad+=2*PI;
  float magDeg=rad2deg(magRad);
  return fmod(magDeg + DECLINATION + 360.0, 360.0);
}
// Reads ultrasonic with media filter-more stable
uint16_t ultrasonicReadCM(){
  unsigned int uS = sonar.ping_median(3);
  if(uS == 0) return US_MAX_DISTANCE_CM + 1;
  return uS / US_ROUNDTRIP_CM;
}

// ================== Motor Control close loop ==================
void motorsStop(){ digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); analogWrite(ENA, 0); analogWrite(ENB, 0); }
void motorsForward(uint8_t pwm){ analogWrite(ENA, pwm); analogWrite(ENB, pwm); digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); }
void motorsTurnLeft(uint8_t pwm){ analogWrite(ENA, pwm); analogWrite(ENB, pwm); digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); }
void motorsTurnRight(uint8_t pwm){ analogWrite(ENA, pwm); analogWrite(ENB, pwm); digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); }
void motorsBackward(uint8_t pwm){ analogWrite(ENA, pwm); analogWrite(ENB, pwm); digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }

// ================== MQTT Callback- REMOTE CONTROL==================
void mqttCallback(char* topic, byte* message, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)message[i];
  msg.trim(); msg.toUpperCase();

  if (String(topic) == TOPIC_CMD_START && (msg == "1" || msg == "START" || msg == "ON")) {
    runEnabled = true;
    mqttClient.publish(TOPIC_STATUS, "ROBOT STARTED");
  } 
  else if (String(topic) == TOPIC_CMD_STOP && (msg == "1" || msg == "STOP" || msg == "OFF")) {
    runEnabled = false; motorsStop();
    mqttClient.publish(TOPIC_STATUS, "ROBOT STOPPED");
  }
  else if (String(topic) == TOPIC_WP_A) { targetWP = 0; mqttClient.publish(TOPIC_STATUS, "WAYPOINT A SELECTED"); }
  else if (String(topic) == TOPIC_WP_B) { targetWP = 1; mqttClient.publish(TOPIC_STATUS, "WAYPOINT B SELECTED"); }
  else if (String(topic) == TOPIC_WP_C) { targetWP = 2; mqttClient.publish(TOPIC_STATUS, "WAYPOINT C SELECTED"); }
}

void connectToWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting WiFi");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\nWiFi Connected!");
}

void reconnectMQTT() {
  while (!mqttClient.connected()) {
    Serial.println("Connecting MQTT...");
    if (mqttClient.connect(MQTT_CLIENTID)) {
      Serial.println("MQTT Connected!");
      mqttClient.subscribe(TOPIC_CMD_START);
      mqttClient.subscribe(TOPIC_CMD_STOP);
      mqttClient.subscribe(TOPIC_WP_A);
      mqttClient.subscribe(TOPIC_WP_B);
      mqttClient.subscribe(TOPIC_WP_C);
    } else delay(2000);
  }
}

// ================== Servo Scan ==================
uint16_t scanDirection(bool left) {
  uint16_t minDist = 999;
  if (left) {
    for (int pos = SERVO_CENTER; pos <= SERVO_LEFT; pos += 15) {
      pan.write(pos); delay(80);
      uint16_t d = ultrasonicReadCM();
      if (d < minDist) minDist = d;
    }
  } else {
    for (int pos = SERVO_CENTER; pos >= SERVO_RIGHT; pos -= 15) {
      pan.write(pos); delay(80);
      uint16_t d = ultrasonicReadCM();
      if (d < minDist) minDist = d;
    }
  }
  pan.write(SERVO_CENTER);
  return minDist;
}

// ================== Setup ==================
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  connectToWiFi();
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  reconnectMQTT();

  compass.init();
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pan.attach(SERVO_PIN);
  pan.write(SERVO_CENTER);
  motorsStop();
  Serial.println("System Ready.");
}

// ================== Main Loop- CLOSED LOOP PID NAVIFATION ==================
void loop() {
  if (WiFi.status() != WL_CONNECTED) connectToWiFi();
  if (!mqttClient.connected()) reconnectMQTT();
  mqttClient.loop();

  if (!runEnabled || targetWP == -1) { motorsStop(); return; }

  while (gpsSerial.available()) gps.encode(gpsSerial.read());
  if (!gps.location.isValid()) { Serial.println("Waiting GPS fix..."); delay(500); return; }

  //Get current position and navigation  data
  double lat = gps.location.lat();
  double lon = gps.location.lng();
  double dist = haversine_m(lat, lon, WPS[targetWP].lat, WPS[targetWP].lon);
  double bearing = bearing_deg(lat, lon, WPS[targetWP].lat, WPS[targetWP].lon);
  float heading = readHeadingTrueDeg();
  float error = wrap180(bearing - heading);
  uint16_t frontDist = ultrasonicReadCM();

  Serial.printf("[NAV] Target=%s Dist=%.2fm Bearing=%.1f° Heading=%.1f° Err=%.1f° Front=%ucm\n",
                WPS[targetWP].name, dist, bearing, heading, error, frontDist);

  // ============ OBSTACLE DETECTED ============
  if (frontDist < US_BLOCK_CM) {
    mqttClient.publish(TOPIC_STATUS, "OBSTACLE DETECTED");
    Serial.println("[EVENT] Obstacle detected -> entering AVOID loop");
    motorsStop();

    int attempts = 0;
    while (true) {
      mqttClient.loop();
      uint16_t leftClear = scanDirection(true);
      uint16_t rightClear = scanDirection(false);
      Serial.printf("[AVOID] L=%ucm R=%ucm (attempt %d)\n", leftClear, rightClear, attempts+1);

      if (leftClear <= US_BLOCK_CM && rightClear <= US_BLOCK_CM) {
        
        motorsBackward(120); delay(700);
        motorsStop(); delay(200);
        attempts++;
        continue;
      }

      mqttClient.publish(TOPIC_STATUS, "AVOIDING");
      if (leftClear > rightClear) motorsTurnLeft(130);
      else motorsTurnRight(130);
      delay(650);

      motorsForward(150); delay(500);
      motorsStop(); delay(200);
      uint16_t postNudge = ultrasonicReadCM();
      if (postNudge > US_CLEAR_CM) {
        
        Serial.println("[AVOID] Path clear -> resuming NAV");
        break;
      }
      motorsBackward(120); delay(800);
      motorsStop(); delay(200);
      attempts++;
    }
    pan.write(SERVO_CENTER);
    delay(150);
    return;
  }

  // ============ WAYPOINT REACHED ============
  if (dist < ARRIVE_M) {
    motorsStop();
    char msg[50];
    sprintf(msg, "REACHED WAYPOINT %s", WPS[targetWP].name);
    mqttClient.publish(TOPIC_STATUS, msg);
    Serial.printf("[NAV] Reached waypoint %s\n", WPS[targetWP].name);
    targetWP = -1;
    return;
  }

  // ============ NAVIGATION CONTROL ============
   const float HEADING_THRESHOLD = 10.0;
  if (error > HEADING_THRESHOLD) {
    motorsTurnLeft(80);
    Serial.println("Turning left");
  }
  else if (error < -HEADING_THRESHOLD) {
    motorsTurnRight(80);
    Serial.println("Turning right");
  }
  else {
    motorsForward(100);
    Serial.println("Moving forward");
  }

  delay(80);

  delay(80);
}
