/*
  Raspberry Pi Pico W - SMART ACTUATOR (CORRECTED)
  Controls: Stepper Motor (Blinds), LED 15 (Lamp), LED 14 (Heater)
*/

#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <LittleFS.h>

// ---------------- CONFIG ----------------
const char *ssid = "alfredo-Linux";
const char *password = "Af4HZ2BG";

const char *mqtt_server = "4979254f05ea480283d67c6f0d9f7525.s1.eu.hivemq.cloud";
const char *mqtt_username = "web_client";
const char *mqtt_password = "Password1";
const int mqtt_port = 8883;

const char *actuation_topic = "/cslab/g01/actuation";

// Hardware Pins
const int IN1 = 28;
const int IN2 = 27;
const int IN3 = 26;
const int IN4 = 22;
const int LED_HEATER = 14;
const int LED_LAMP = 15;

// Motor Config
const int MAX_POS = 9216; // Full extension
// --- FIX WAS HERE: Changed 'int' to 'const char *' ---
const char *POS_FILE_PATH = "/motor_pos.txt"; 

const int stepCount = 8;
const int halfStepSequence[stepCount][4] = {
  {1, 0, 0, 0}, {1, 1, 0, 0}, {0, 1, 0, 0}, {0, 1, 1, 0},
  {0, 0, 1, 0}, {0, 0, 1, 1}, {0, 0, 0, 1}, {1, 0, 0, 1}
};

// State
int currentStepPhase = 0;
long absolutePosition = 0; // 0 = Closed, MAX_POS = Open
WiFiClientSecure picoClient;
PubSubClient client(picoClient);

// ---------------- HELPERS ----------------

void savePosition() {
  File f = LittleFS.open(POS_FILE_PATH, "w");
  if (f) {
    f.printf("%ld %d", absolutePosition, currentStepPhase);
    f.close();
  }
}

void loadPosition() {
  if (LittleFS.exists(POS_FILE_PATH)) {
    File f = LittleFS.open(POS_FILE_PATH, "r");
    if (f) {
      absolutePosition = f.parseInt();
      currentStepPhase = f.parseInt();
      f.close();
    }
  }
}

void stepMotor(int steps) {
  int direction = (steps >= 0) ? 1 : -1;
  int stepsLeft = abs(steps);

  while (stepsLeft > 0) {
    // BOUNDS CHECK
    if (direction == 1 && absolutePosition >= MAX_POS) break;
    if (direction == -1 && absolutePosition <= 0) break;

    currentStepPhase = (currentStepPhase + direction + stepCount) % stepCount;
    absolutePosition += direction;

    digitalWrite(IN1, halfStepSequence[currentStepPhase][0]);
    digitalWrite(IN2, halfStepSequence[currentStepPhase][1]);
    digitalWrite(IN3, halfStepSequence[currentStepPhase][2]);
    digitalWrite(IN4, halfStepSequence[currentStepPhase][3]);

    delay(3); // Speed
    
    // CRITICAL: Keep MQTT alive during long motor moves
    if (stepsLeft % 100 == 0) client.loop(); 
    
    stepsLeft--;
  }
  
  // Cut power to coils
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  
  savePosition();
}

// ---------------- ACTUATION LOGIC ----------------

void handleLightCommand(bool turnOn) {
  if (turnOn) {
    Serial.println("CMD: Increase Light");
    // Priority 1: Open Blinds
    if (absolutePosition < MAX_POS) {
      Serial.println("Action: Opening Blinds...");
      stepMotor(MAX_POS - absolutePosition); // Move to Max
    } 
    // Priority 2: If Blinds Open, Turn on Lamp
    else {
      Serial.println("Action: Blinds Open. Turning on Lamp.");
      digitalWrite(LED_LAMP, HIGH);
    }
  } else {
    Serial.println("CMD: Decrease Light");
    // Priority 1: Turn off Lamp
    digitalWrite(LED_LAMP, LOW);
    
    // Priority 2: Close Blinds
    if (absolutePosition > 0) {
      Serial.println("Action: Closing Blinds...");
      stepMotor(-absolutePosition); // Move to 0
    }
  }
}

void handleHeatCommand(bool turnOn) {
  digitalWrite(LED_HEATER, turnOn ? HIGH : LOW);
  Serial.print("CMD: Heater ");
  Serial.println(turnOn ? "ON" : "OFF");
}

// ---------------- MQTT CALLBACK ----------------

void callback(char *topic, byte *payload, unsigned int length) {
  char msg[length + 1];
  memcpy(msg, payload, length);
  msg[length] = '\0';
  Serial.print("Msg: "); Serial.println(msg);

  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, msg);
  if (error) return;

  const char* type = doc["type"];   
  const char* action = doc["action"]; 
  
  if (!type || !action) return; // Safety check

  bool turnOn = (strcmp(action, "ON") == 0);

  if (strcmp(type, "LIGHT") == 0) {
    handleLightCommand(turnOn);
  } else if (strcmp(type, "HEAT") == 0) {
    handleHeatCommand(turnOn);
  }
}

// ---------------- SETUP & LOOP ----------------

void setup() {
  Serial.begin(115200);
  
  // Pins
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(LED_HEATER, OUTPUT); pinMode(LED_LAMP, OUTPUT);

  // FS & Position
  if (!LittleFS.begin()) { LittleFS.format(); LittleFS.begin(); }
  loadPosition();

  // WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);
  Serial.println("Actuator Online.");

  // MQTT
  picoClient.setInsecure();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) {
    String clientId = "PICO_Actuator-" + String(random(0xffff), HEX);
    if (client.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
      client.subscribe(actuation_topic);
      Serial.println("Subscribed.");
    } else {
      delay(5000);
    }
  }
  client.loop();
}