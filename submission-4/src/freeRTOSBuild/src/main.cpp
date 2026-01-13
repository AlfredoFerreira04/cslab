// Raspberry Pi Pico W - FreeRTOS Worker
// Reads LDR (GP26) and DHT11 (GP4) -> Sends to Server via Queue

#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

#include <WiFi.h>
#include <WiFiUdp.h>
#include <DHT.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>

// ---------------- CONFIG ----------------
const char *ssid = "Pixel_Alf";
const char *password = "alfredopassword04";

const IPAddress udp_server_ip(10, 199, 121, 191);
const int udp_port = 5005;

const char *mqtt_server = "4979254f05ea480283d67c6f0d9f7525.s1.eu.hivemq.cloud";
const char *mqtt_username = "web_client";
const char *mqtt_password = "Password1";
const int mqtt_port = 8883;

const char *mqtt_command_topic = "/cslab/g01/commands"; 
const char *mqtt_actuator_topic = "/cslab/g01/actuation"; 

#define DHTPIN 4
#define DHTTYPE DHT11
#define LDR_PIN 26 
#define DEVICE_ID "PICO_Device_01" 
#define MAX_RETRIES 3

// ---------------- OBJECTS & HANDLES ----------------
DHT dht(DHTPIN, DHTTYPE);
WiFiUDP udp;
WiFiClientSecure picoClient;
PubSubClient client(picoClient);

// Data structure to pass between tasks
struct SensorData {
    float tempReading;
    int lightReading;
};

// RTOS Handles
QueueHandle_t sensorQueueHandle;
SemaphoreHandle_t wifiMutex; // To protect WiFi usage if needed (optional here as we isolate net ops)

// Globals
unsigned long seq = 0;
bool fs_is_ready = false;

// ---------------- HELPER FUNCTIONS ----------------

// Setup File System
void setupLittleFS() {
    if (!LittleFS.begin()) { LittleFS.format(); LittleFS.begin(); }
    fs_is_ready = true;
}

// Write failed sends to file
void logDataToFile(const String &payload) {
    if (!fs_is_ready) return;
    File f = LittleFS.open("/telemetry_log.txt", "a");
    if (f) { f.println(payload); f.close(); }
}

// Read old files and try to send them
void transmitStoredData() {
    if (!fs_is_ready) return;
    if (!LittleFS.exists("/telemetry_log.txt")) return;

    File f = LittleFS.open("/telemetry_log.txt", "r");
    if (!f) return;
    
    // In a real scenario, you would parse line by line and send. 
    // For simplicity, we just check existence and delete after 'processing'
    // or you can implement the line-by-line send here.
    f.close();
    LittleFS.remove("/telemetry_log.txt"); 
}

// UDP ACK Waiter
bool waitForAck(unsigned long mySeq) {
    unsigned long start = millis();
    char incoming[512];
    
    // Wait up to 500ms for ACK
    while (millis() - start < 500) { 
        int packetSize = udp.parsePacket();
        if (packetSize) {
            int len = udp.read(incoming, sizeof(incoming)-1);
            incoming[len] = 0;
            StaticJsonDocument<128> doc;
            if (!deserializeJson(doc, incoming)) {
                if (doc["id"] == DEVICE_ID && doc["seq"] == mySeq && doc["type"] == "ACK") return true;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // Yield to other tasks while waiting
    }
    return false;
}

// Send UDP with Retries
bool sendWithQoS(const String &payload, unsigned long current_seq) {
    if (WiFi.status() != WL_CONNECTED) return false;
    int retry = 0;
    while (retry < MAX_RETRIES) {
        udp.beginPacket(udp_server_ip, udp_port);
        udp.print(payload);
        udp.endPacket();
        
        if (waitForAck(current_seq)) return true;
        
        retry++;
        vTaskDelay(pdMS_TO_TICKS(100 * retry)); // RTOS non-blocking delay
    }
    return false;
}

void relayToActuator(const char* type, const char* action) {
    if (!client.connected()) return;
    StaticJsonDocument<200> doc;
    doc["type"] = type;     
    doc["action"] = action; 
    char buffer[256];
    serializeJson(doc, buffer);
    client.publish(mqtt_actuator_topic, buffer);
}

void mqttCallback(char *topic, byte *payload, unsigned int length) {
    char msg[length + 1];
    memcpy(msg, payload, length);
    msg[length] = '\0';
    
    if (strcmp(topic, mqtt_command_topic) == 0) {
        StaticJsonDocument<256> doc;
        DeserializationError error = deserializeJson(doc, msg);
        if (error) return;

        const char* target = doc["target_id"];
        if (target && strcmp(target, DEVICE_ID) == 0) {
            relayToActuator(doc["type"], doc["action"]);
        }
    }
}

void reconnectMqtt() {
    if (client.connected()) return;
    String clientId = "PICO_Worker-" + String(random(0xffff), HEX);
    if (client.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
        client.subscribe(mqtt_command_topic);
        Serial.println("MQTT Connected");
    }
}

// ---------------- TASKS ----------------

// Task 1: Read Sensors every 1 second
void SensorTask(void *pvParameters) {
    (void) pvParameters;

    for (;;) {
        SensorData data;
        
        // Read Sensors
        data.tempReading = dht.readTemperature();
        int rawLdr = analogRead(LDR_PIN);
        data.lightReading = 4095 - rawLdr; // Invert logic

        if (isnan(data.tempReading)) data.tempReading = 0.0;

        // Send to Queue
        // portMAX_DELAY will wait indefinitely if queue is full, 
        // or use 0 to drop packet if full.
        xQueueSend(sensorQueueHandle, &data, portMAX_DELAY);

        // Run this task every 1000ms
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Task 2: Handle Network (WiFi, MQTT, UDP)
void NetworkTask(void *pvParameters) {
    (void) pvParameters;

    // Initial Connection
    Serial.print("Connecting to WiFi");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        vTaskDelay(pdMS_TO_TICKS(500));
        Serial.print(".");
    }
    Serial.println("\nWiFi Connected");

    udp.begin(udp_port);
    picoClient.setInsecure();
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(mqttCallback);

    for (;;) {
        // 1. Maintain Connectivity (The Fix)
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("WiFi lost. Reconnecting...");
            WiFi.disconnect(); // Clear invalid state
            WiFi.begin(ssid, password); // Try connecting again
            
            // Wait a bit before checking or trying again to prevent flooding
            vTaskDelay(pdMS_TO_TICKS(2000)); 
            continue; 
        }

        if (!client.connected()) reconnectMqtt();
        client.loop(); // Must be called frequently

        // 2. Check Queue for new Sensor Data
        SensorData receivedData;
        if (xQueueReceive(sensorQueueHandle, &receivedData, pdMS_TO_TICKS(100)) == pdTRUE) {
            
            StaticJsonDocument<256> doc;
            doc["id"] = DEVICE_ID;
            doc["type"] = "WeatherObserved";
            doc["temperature"] = receivedData.tempReading;
            doc["light"] = receivedData.lightReading;
            doc["dateObserved"] = millis();
            doc["qos"] = 1;
            doc["seq"] = seq;

            String payload;
            serializeJson(doc, payload);

            bool delivered = sendWithQoS(payload, seq);
            
            if (client.connected()) {
                client.publish("/cslab/g01/sensor", payload.c_str(), true);
            }

            if (!delivered) {
                Serial.println("Send failed, logging to FS");
                logDataToFile(payload);
            } else {
                transmitStoredData();
            }
            seq++;
        }
    }
}

// ---------------- SETUP ----------------
void setup() {
    Serial.begin(115200);
    analogReadResolution(12);
    setupLittleFS();
    dht.begin();

    // Create Queue (Depth of 20 items)
    sensorQueueHandle = xQueueCreate(20, sizeof(SensorData));

    // Create Tasks
    // Stack sizes: 2048 words is usually sufficient for simple tasks, 
    // but WiFi/SSL needs more (4096 or 8192).
    xTaskCreate(SensorTask, "Sensor Task", 2048, NULL, 1, NULL);
    
    // Network task gets higher stack because of SSL/WiFi usage
    xTaskCreate(NetworkTask, "Network Task", 8192, NULL, 1, NULL);

    // Note: In Arduino Pico Core (Earle Philhower), the scheduler 
    // is started automatically after setup() finishes.
}

void loop() {
    // Empty. The tasks are doing the work.
}