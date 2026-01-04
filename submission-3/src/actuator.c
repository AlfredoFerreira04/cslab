/*
  Raspberry Pi Pico W - SMART ACTUATOR (QUEUED ARCHITECTURE)
  Controls: Stepper Motor (Blinds), LED 15 (Lamp), LED 14 (Heater)
  Logic: uses isBlindsClosed (1=Closed, 0=Open).
  Feature: Circular Queue (Size 5) + Mutex for safe processing.
*/

#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <LittleFS.h>

// ---------------- CONFIG ----------------
const char *ssid = "alfredo-Linux";
const char *password = "nhQXMozZ";

const char *mqtt_server = "4979254f05ea480283d67c6f0d9f7525.s1.eu.hivemq.cloud";
const char *mqtt_username = "web_client";
const char *mqtt_password = "Password1";
const int mqtt_port = 8883;

const char *actuation_topic = "/cslab/g01/commands";

// Hardware Pins
const int IN1 = 28;
const int IN2 = 27;
const int IN3 = 26;
const int IN4 = 22;
const int LED_HEATER = 14;
const int LED_LAMP = 15;

// Motor Config
const int MAX_POS = 9216; // Steps for full open/close
const char *STATE_FILE_PATH = "/motor_state_v2.txt"; 

const int stepCount = 8;
const int halfStepSequence[stepCount][4] = {
  {1, 0, 0, 0}, {1, 1, 0, 0}, {0, 1, 0, 0}, {0, 1, 1, 0},
  {0, 0, 1, 0}, {0, 0, 1, 1}, {0, 0, 0, 1}, {1, 0, 0, 1}
};

// State Variables
int isBlindsClosed = 1; 
int currentStepPhase = 0; 

// ---------------- QUEUE & MUTEX DEFINITIONS ----------------
const int QUEUE_SIZE = 5;
bool lightActionQueue[QUEUE_SIZE]; // Stores 'turnOn' (true/false)
int queueHead = 0; // Where we read from (Oldest)
int queueTail = 0; // Where we write to (Newest)
int queueCount = 0;

// The Mutex (Locked = true, Unlocked = false)
bool motorMutex = false; 

WiFiClientSecure picoClient;
PubSubClient client(picoClient);

// ---------------- QUEUE HELPERS ----------------

void enqueueLightCommand(bool turnOn) {
  // If full, we drop the oldest to make room for the new one (Overwrite strategy)
  if (queueCount == QUEUE_SIZE) {
      Serial.println("QUEUE: Full. Overwriting oldest command.");
      queueHead = (queueHead + 1) % QUEUE_SIZE; // Advance head (drop oldest)
      queueCount--;
  }
  
  lightActionQueue[queueTail] = turnOn;
  queueTail = (queueTail + 1) % QUEUE_SIZE;
  queueCount++;
  
  Serial.print("QUEUE: Added command. Count: "); Serial.println(queueCount);
}

// Returns true if an item was retrieved, false if empty
bool dequeueLightCommand(bool &actionContainer) {
  if (queueCount == 0) return false;
  
  actionContainer = lightActionQueue[queueHead];
  queueHead = (queueHead + 1) % QUEUE_SIZE;
  queueCount--;
  return true;
}

// ---------------- PERSISTENCE ----------------

void saveState() {
  File f = LittleFS.open(STATE_FILE_PATH, "w");
  if (f) {
    f.printf("%d %d", isBlindsClosed, currentStepPhase);
    f.close();
    Serial.println("FS: State Saved.");
  }
}

void loadState() {
  if (LittleFS.exists(STATE_FILE_PATH)) {
    File f = LittleFS.open(STATE_FILE_PATH, "r");
    if (f) {
      isBlindsClosed = f.parseInt();
      currentStepPhase = f.parseInt();
      f.close();
      Serial.print("FS: Loaded State. isBlindsClosed = ");
      Serial.println(isBlindsClosed);
    }
  } else {
    Serial.println("FS: No save file. Defaulting to CLOSED (1).");
    isBlindsClosed = 1;
  }
}

// ---------------- MOTOR LOGIC ----------------

void stepMotor(int steps) {
  Serial.print("MOTOR: Moving steps: "); Serial.println(steps);
  int direction = (steps >= 0) ? 1 : -1;
  int stepsLeft = abs(steps);

  while (stepsLeft > 0) {
    currentStepPhase = (currentStepPhase + direction + stepCount) % stepCount;

    digitalWrite(IN1, halfStepSequence[currentStepPhase][0]);
    digitalWrite(IN2, halfStepSequence[currentStepPhase][1]);
    digitalWrite(IN3, halfStepSequence[currentStepPhase][2]);
    digitalWrite(IN4, halfStepSequence[currentStepPhase][3]);

    delay(3); // Speed
    
    // Check for incoming messages (Will go to Queue, won't interrupt this movement)
    if (stepsLeft % 100 == 0) client.loop(); 
    stepsLeft--;
  }
  
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  Serial.println("MOTOR: Movement Complete.");
}

// ---------------- ACTUATION LOGIC (CONSUMER) ----------------

void handleLightCommand(bool turnOn) {
  // 1. LOCK MUTEX
  Serial.println("MUTEX: Locked.");
  motorMutex = true; 

  // --- CRITICAL SECTION START ---
  if (turnOn) {
    // CMD: INCREASE LIGHT (ON)
    Serial.println("CMD: Increase Light (ON)");
    if (isBlindsClosed == 1) {
      Serial.println("Action: Opening Blinds...");
      stepMotor(MAX_POS); 
      isBlindsClosed = 0; 
      saveState();
    } else {
      Serial.println("Action: Blinds already OPEN. Lamp ON.");
      digitalWrite(LED_LAMP, HIGH);
    }
  } 
  else {
    // CMD: DECREASE LIGHT (OFF)
    Serial.println("CMD: Decrease Light (OFF)");
    digitalWrite(LED_LAMP, LOW);
    
    if (isBlindsClosed == 0) {
      Serial.println("Action: Closing Blinds...");
      stepMotor(-MAX_POS); 
      isBlindsClosed = 1; 
      saveState(); 
    } else {
      Serial.println("Action: Blinds already CLOSED.");
    }
  }
  // --- CRITICAL SECTION END ---

  // 2. UNLOCK MUTEX
  motorMutex = false;
  Serial.println("MUTEX: Unlocked.");
}

void handleHeatCommand(bool turnOn) {
  // Heater is instant (no motor), so it doesn't strictly need the queue/mutex 
  // unless you want to prevent it changing while motor moves. 
  // For now, we allow heater to change instantly.
  Serial.print("CMD: Heater "); Serial.println(turnOn ? "ON" : "OFF");
  digitalWrite(LED_HEATER, turnOn ? HIGH : LOW);
}

// ---------------- MQTT CALLBACK (PRODUCER) ----------------

void callback(char *topic, byte *payload, unsigned int length) {
  char msg[length + 1];
  memcpy(msg, payload, length);
  msg[length] = '\0';
  Serial.print("MQTT Received: "); Serial.println(msg);

  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, msg);
  if (error) return;

  const char* type = doc["type"];   
  const char* action = doc["action"]; 
  if (!type || !action) return;

  bool turnOn = (strcmp(action, "ON") == 0);

  if (strcmp(type, "LIGHT") == 0) {
    // DO NOT execute logic here. Just Add to Queue.
    enqueueLightCommand(turnOn);
  } 
  else if (strcmp(type, "HEAT") == 0) {
    handleHeatCommand(turnOn);
  }
}

// ---------------- SETUP & LOOP ----------------

void setup() {
  Serial.begin(115200);
  
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(LED_HEATER, OUTPUT); pinMode(LED_LAMP, OUTPUT);
  digitalWrite(LED_HEATER, LOW); digitalWrite(LED_LAMP, LOW);

  if (!LittleFS.begin()) { LittleFS.format(); LittleFS.begin(); }
  loadState();

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);
  Serial.println("Actuator Online.");

  picoClient.setInsecure();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void loop() {
  // 1. Maintain Network
  if (!client.connected()) {
    String clientId = "PICO_Actuator-" + String(random(0xffff), HEX);
    if (client.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
      client.subscribe(actuation_topic);
      Serial.println("Subscribed.");
    } else { delay(5000); }
  }
  client.loop(); // This checks for new packets -> triggers callback -> fills queue

  // 2. Process Queue (Consumer)
  // Logic: Notice queue has items -> Try locking -> If free, Execute oldest
  if (queueCount > 0) {
    if (!motorMutex) {
      // Mutex is FREE. We can proceed.
      bool actionToPerform;
      if (dequeueLightCommand(actionToPerform)) {
         handleLightCommand(actionToPerform);
      }
    } 
    // If motorMutex is TRUE, we just loop again (non-blocking wait)
  }
}