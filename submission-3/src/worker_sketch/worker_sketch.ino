// Raspberry Pi Pico W - FreeRTOS Worker
// Environment: Arduino Mbed OS or Earle Philhower Core (supports FreeRTOS standard)
#define __FREERTOS 1
#include <WiFi.h>
#include <WiFiUdp.h>
#include <DHT.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>

// FreeRTOS headers are usually auto-included in Pico Arduino cores, 
// but strictly speaking:
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

// ---------------- CONFIG ----------------
const char *ssid = "alfredo-Linux";
const char *password = "nhQXMozZ";

const IPAddress udp_server_ip(10, 42, 0, 1);
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

// ---------------- GLOBALS & HANDLES ----------------

DHT dht(DHTPIN, DHTTYPE);
WiFiUDP udp;
WiFiClientSecure picoClient;
PubSubClient client(picoClient);

// RTOS Handles
QueueHandle_t sensorQueue;
SemaphoreHandle_t wifiMutex; // Optional: To protect WiFi if multiple tasks use it

struct SensorData {
    float tempReading;
    int lightReading;
    unsigned long timestamp;
};

unsigned long seq = 0;
bool fs_is_ready = false;

// ---------------- HELPERS (Network Logic) ----------------

// Helper: Setup FS
void setupLittleFS() {
    if (!LittleFS.begin()) { LittleFS.format(); LittleFS.begin(); }
    fs_is_ready = true;
}

// Helper: Read LDR
int readLDR() {   
    int rawValue = analogRead(LDR_PIN);
    return 4095 - rawValue; 
}

// Helper: MQTT Callback
void relayToActuator(const char* type, const char* action) {
    if (!client.connected()) return;
    StaticJsonDocument<200> doc;
    doc["type"] = type;     
    doc["action"] = action; 
    char buffer[256];
    serializeJson(doc, buffer);
    client.publish(mqtt_actuator_topic, buffer);
}

void callback(char *topic, byte *payload, unsigned int length) {
    char msg[length + 1];
    memcpy(msg, payload, length);
    msg[length] = '\0';
    
    StaticJsonDocument<256> doc;
    if (deserializeJson(doc, msg)) return;

    if (doc["target_id"] && strcmp(doc["target_id"], DEVICE_ID) == 0) {
        relayToActuator(doc["type"], doc["action"]);
    }
}

void reconnectMqtt() {
    // Non-blocking attempt approach usually preferred in RTOS, 
    // but for simplicity we keep a short blocking retry loop here.
    if (!client.connected()) {
        String clientId = "PICO_Worker-" + String(random(0xffff), HEX);
        if (client.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
            client.subscribe(mqtt_command_topic);
        }
    }
}

// Helper: UDP ACK Wait
bool waitForAck(unsigned long mySeq) {
    unsigned long start = millis();
    char incoming[512];
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
        // Very important: Yield to other tasks while waiting!
        vTaskDelay(10 / portTICK_PERIOD_MS); 
    }
    return false;
}

bool sendWithQoS(const String &payload, unsigned long current_seq) {
    if (WiFi.status() != WL_CONNECTED) return false;
    for(int retry=0; retry<3; retry++) {
        udp.beginPacket(udp_server_ip, udp_port);
        udp.print(payload);
        udp.endPacket();
        if (waitForAck(current_seq)) return true;
        vTaskDelay(100 * (retry + 1) / portTICK_PERIOD_MS); 
    }
    return false;
}

void logDataToFile(const String &payload) {
    if (!fs_is_ready) return;
    File f = LittleFS.open("/telemetry_log.txt", "a");
    if (f) { f.println(payload); f.close(); }
}

void transmitStoredData() {
    if (!fs_is_ready) return;
    if (!LittleFS.exists("/telemetry_log.txt")) return;

    File f = LittleFS.open("/telemetry_log.txt", "r");
    if (!f) return;
    
    // In a real RTOS app, you might read line by line and send, 
    // rather than dumping everything at once which might block too long.
    // For now, we assume simple file existence check.
    f.close();
    LittleFS.remove("/telemetry_log.txt"); 
}

// ---------------- TASKS ----------------

// Task 1: Sensor Reading (Producer)
// Runs on Core 0 or 1, handles hardware timing strictly.
void TaskSensors(void *pvParameters) {
    (void) pvParameters;

    for (;;) {
        SensorData data;
        
        // 1. Read Hardware
        data.tempReading = dht.readTemperature();
        data.lightReading = readLDR();
        data.timestamp = millis();

        if (isnan(data.tempReading)) data.tempReading = 0.0;

        // 2. Send to Queue
        // wait up to 10ms if queue is full
        xQueueSend(sensorQueue, &data, 10 / portTICK_PERIOD_MS);

        // 3. Block for 1 second (1000ms)
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

// Task 2: Network & Comm (Consumer)
// Handles WiFi, MQTT loop, and draining the queue to UDP.
void TaskNetwork(void *pvParameters) {
    (void) pvParameters;

    // Connect WiFi inside the task (or in setup)
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    
    udp.begin(udp_port);
    picoClient.setInsecure();
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callback);

    SensorData data;
    
    for (;;) {
        // Maintain MQTT connection
        if (!client.connected()) reconnectMqtt();
        client.loop();

        // Check Queue for new sensor data
        // portMAX_DELAY means "sleep until something arrives"
        // But we use a short timeout so we can keep running client.loop()
        if (xQueueReceive(sensorQueue, &data, 100 / portTICK_PERIOD_MS) == pdPASS) {
            
            // Construct Payload
            StaticJsonDocument<256> doc;
            doc["id"] = DEVICE_ID;
            doc["type"] = "WeatherObserved";
            doc["temperature"] = data.tempReading;
            doc["light"] = data.lightReading;
            doc["dateObserved"] = data.timestamp;
            doc["qos"] = 1;
            doc["seq"] = seq;

            String payload;
            serializeJson(doc, payload);

            // Send
            bool delivered = sendWithQoS(payload, seq);
            if (client.connected()) client.publish("/comcs/g04/sensor", payload.c_str(), true);

            // Handle Persistence
            if (!delivered) logDataToFile(payload);
            else transmitStoredData();

            seq++;
        }
        
        // Small yield not strictly necessary if QueueReceive blocks, 
        // but good for safety in some RTOS configs.
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

// ---------------- MAIN ----------------

void setup() {
    Serial.begin(115200);
    analogReadResolution(12);
    setupLittleFS();
    dht.begin();

    // Create a queue capable of holding 20 sensor readings
    sensorQueue = xQueueCreate(20, sizeof(SensorData));

    // Create Tasks
    // Stack sizes (2048/4096) may need adjustment based on library usage
    xTaskCreate(TaskSensors, "Sensors", 2048, NULL, 1, NULL);
    xTaskCreate(TaskNetwork, "Network", 8192, NULL, 1, NULL); // Bigger stack for WiFi/SSL

    // Start Scheduler (In Arduino Pico, this happens automatically after setup)
}

void loop() {
    // In FreeRTOS, the loop is usually empty or used for low-priority idle work.
    // The real work happens in the tasks above.
    vTaskDelete(NULL); // Kill the loop task to free memory
}