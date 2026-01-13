/*
  Raspberry Pi Pico W - SMART ACTUATOR (FreeRTOS VERSION)
  Architecture:
   - NetworkTask (Core 0): Handles WiFi/MQTT, pushes commands to Queue.
   - MotorTask (Core 1): Pops commands from Queue, drives Stepper Motor.
*/

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <LittleFS.h>

// ---------------- CONFIG ----------------
const char *ssid = "Pixel_Alf";
const char *password = "alfredopassword04";

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
const int MAX_POS = 11254; 
const char *STATE_FILE_PATH = "/motor_state_v2.txt"; 

const int stepCount = 8;
const int halfStepSequence[stepCount][4] = {
  {1, 0, 0, 0}, {1, 1, 0, 0}, {0, 1, 0, 0}, {0, 1, 1, 0},
  {0, 0, 1, 0}, {0, 0, 1, 1}, {0, 0, 0, 1}, {1, 0, 0, 1}
};

// ---------------- GLOBAL STATE ----------------
// Protected by xMotorMutex logic implicitly via single-consumer task
int isBlindsClosed = 1; 
int currentStepPhase = 0; 

// FreeRTOS Handles
QueueHandle_t xLightCommandQueue;
SemaphoreHandle_t xFileSystemMutex; // To prevent FS corruption
TaskHandle_t wifiTaskHandle = NULL;
TaskHandle_t motorTaskHandle = NULL;

WiFiClientSecure picoClient;
PubSubClient client(picoClient);

// ---------------- PERSISTENCE (Thread-Safe) ----------------

void saveState() {
  if (xSemaphoreTake(xFileSystemMutex, portMAX_DELAY) == pdTRUE) {
    File f = LittleFS.open(STATE_FILE_PATH, "w");
    if (f) {
      f.printf("%d %d", isBlindsClosed, currentStepPhase);
      f.close();
      Serial.println("FS: State Saved.");
    }
    xSemaphoreGive(xFileSystemMutex);
  }
}

void loadState() {
  if (xSemaphoreTake(xFileSystemMutex, portMAX_DELAY) == pdTRUE) {
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
    xSemaphoreGive(xFileSystemMutex);
  }
}

// ---------------- MOTOR LOGIC ----------------

void stepMotorRaw(int steps) {
  int direction = (steps >= 0) ? 1 : -1;
  int stepsLeft = abs(steps);

  while (stepsLeft > 0) {
    currentStepPhase = (currentStepPhase + direction + stepCount) % stepCount;
    
    digitalWrite(IN1, halfStepSequence[currentStepPhase][0]);
    digitalWrite(IN2, halfStepSequence[currentStepPhase][1]);
    digitalWrite(IN3, halfStepSequence[currentStepPhase][2]);
    digitalWrite(IN4, halfStepSequence[currentStepPhase][3]);

    // vTaskDelay is non-blocking to other tasks!
    vTaskDelay(pdMS_TO_TICKS(3)); 
    stepsLeft--;
  }
  
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

// ---------------- MQTT CALLBACK ----------------

void mqttCallback(char *topic, byte *payload, unsigned int length) {
  char msg[length + 1];
  memcpy(msg, payload, length);
  msg[length] = '\0';
  Serial.print("MQTT: "); Serial.println(msg);

  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, msg);
  if (error) return;

  const char* type = doc["type"];   
  const char* action = doc["action"]; 
  if (!type || !action) return;

  bool turnOn = (strcmp(action, "ON") == 0);

  if (strcmp(type, "LIGHT") == 0) {
    // Send to Queue. Don't block if full, just drop or overwrite.
    // Here we wait 0 ticks (non-blocking).
    if (xQueueSend(xLightCommandQueue, &turnOn, 0) != pdPASS) {
      Serial.println("QUEUE: Full! Command dropped.");
    } else {
      Serial.println("QUEUE: Command Enqueued.");
    }
  } 
  else if (strcmp(type, "HEAT") == 0) {
    // Heater is instant, handle directly in Network Task
    Serial.print("CMD: Heater "); Serial.println(turnOn ? "ON" : "OFF");
    digitalWrite(LED_HEATER, turnOn ? HIGH : LOW);
  }
}

// ---------------- TASKS ----------------

// TASK 1: MOTOR CONSUMER (Runs on Core 1 typically via Scheduler)
void MotorTask(void *parameter) {
  bool turnOn;

  for (;;) {
    // Block indefinitely until a command arrives
    if (xQueueReceive(xLightCommandQueue, &turnOn, portMAX_DELAY) == pdPASS) {
      
      Serial.print("MOTOR TASK: Processing Light Command: "); 
      Serial.println(turnOn ? "ON" : "OFF");

      if (turnOn) {
        // CMD: ON
        if (isBlindsClosed == 1) {
          Serial.println("Action: Opening Blinds...");
          stepMotorRaw(MAX_POS);
          isBlindsClosed = 0;
          saveState();
        } else {
          Serial.println("Action: Blinds Open. Lamp ON.");
          digitalWrite(LED_LAMP, HIGH);
        }
      } 
      else {
        // CMD: OFF
        digitalWrite(LED_LAMP, LOW);
        if (isBlindsClosed == 0) {
          Serial.println("Action: Closing Blinds...");
          stepMotorRaw(-MAX_POS);
          isBlindsClosed = 1;
          saveState();
        } else {
          Serial.println("Action: Blinds already Closed.");
        }
      }
    }
  }
}

// TASK 2: NETWORK PRODUCER (Runs on Core 0)
void NetworkTask(void *parameter) {
  // Connect WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(500));
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected.");

  picoClient.setInsecure();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);

  for (;;) {
    if (!client.connected()) {
      Serial.print("Connecting MQTT...");
      String clientId = "PICO_RTOS-" + String(random(0xffff), HEX);
      if (client.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
        Serial.println("Connected!");
        client.subscribe(actuation_topic);
      } else {
        Serial.print("Failed rc="); Serial.println(client.state());
        vTaskDelay(pdMS_TO_TICKS(5000));
      }
    }
    
    client.loop();
    // Yield to allow other tasks on this core to run (if any)
    vTaskDelay(pdMS_TO_TICKS(10)); 
  }
}

// ---------------- SETUP ----------------

void setup() {
  Serial.begin(115200);
  delay(2000); // Allow Serial Monitor to catch up

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(LED_HEATER, OUTPUT); pinMode(LED_LAMP, OUTPUT);
  digitalWrite(LED_HEATER, LOW); digitalWrite(LED_LAMP, LOW);

  // Initialize File System
  if (!LittleFS.begin()) { LittleFS.format(); LittleFS.begin(); }
  
  // Create Mutex for Thread-Safe File Access
  xFileSystemMutex = xSemaphoreCreateMutex();
  loadState();

  // Create Queue (Size 5, Item Size = sizeof(bool))
  xLightCommandQueue = xQueueCreate(5, sizeof(bool));
  if (xLightCommandQueue == NULL) {
    Serial.println("Error creating the queue");
    while(1);
  }

  Serial.println("Starting FreeRTOS Scheduler...");

  // Create Tasks
  // xTaskCreate(Function, Name, StackSize, Param, Priority, Handle)
  
  // Network Task: High Priority to keep connection alive
  xTaskCreate(NetworkTask, "NetworkTask", 4096, NULL, 2, &wifiTaskHandle);

  // Motor Task: Lower Priority (but will consume CPU when running)
  xTaskCreate(MotorTask, "MotorTask", 4096, NULL, 1, &motorTaskHandle);

  // Note: on Arduino for Pico, vTaskStartScheduler() is called automatically after setup() exits.
}

void loop() {
  // Empty. Everything is handled in Tasks.
  // We can delete the 'loop' task to free up resources if we want,
  // or just yield indefinitely.
  vTaskDelete(NULL); 
}