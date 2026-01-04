// Raspberry Pi Pico W - Worker (Real Sensors: DHT11 + LDR Module)
// Reads LDR (GP26) and DHT11 (GP4) -> Sends to Server
// Relays commands to Actuator

#include <WiFi.h>
#include <WiFiUdp.h>
#include <DHT.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <vector>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>

// ---------------- CONFIG ----------------
const char *ssid = "alfredo-Linux";
const char *password = "nhQXMozZ";

const IPAddress udp_server_ip(10, 42, 0, 1);
const int udp_port = 5005;

const char *mqtt_server = "4979254f05ea480283d67c6f0d9f7525.s1.eu.hivemq.cloud";
const char *mqtt_username = "web_client";
const char *mqtt_password = "Password1";
const int mqtt_port = 8883;

// Topics
const char *mqtt_command_topic = "/cslab/g01/commands"; 
const char *mqtt_actuator_topic = "/cslab/g01/actuation"; 

#define DHTPIN 4
#define DHTTYPE DHT11

// Tutorial uses AO pin connected to GP26 (ADC0)
#define LDR_PIN 26 

#define MAX_RETRIES 3
#define INITIAL_BACKOFF_MS 200

#define DEVICE_ID "PICO_Device_01" 

DHT dht(DHTPIN, DHTTYPE);
WiFiUDP udp;
WiFiClientSecure picoClient;
PubSubClient client(picoClient);

struct SensorData {
    float tempReading;
    int lightReading;
};

std::vector<SensorData> sensorQueue;
unsigned long seq = 0;
unsigned long current_delay = 5000;
bool fs_is_ready = false;
unsigned long lastSensorTime = 0;
unsigned long lastNetworkTime = 0;

// ---------------- HELPERS ----------------

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
    
    if (strcmp(topic, mqtt_command_topic) == 0) {
        StaticJsonDocument<256> doc;
        DeserializationError error = deserializeJson(doc, msg);
        if (error) return;

        const char* target = doc["target_id"];
        const char* type = doc["type"];
        const char* action = doc["action"];

        if (target && strcmp(target, DEVICE_ID) == 0) {
            relayToActuator(type, action);
        }
    }
}

// ---------------- SENSOR READING ----------------

int readLDR() {   
    int rawValue = analogRead(LDR_PIN);
    
    // INVERSION LOGIC for Modules:
    // Modules usually output HIGH (4095) for DARK.
    // Your Server expects LOW (0) for DARK.
    // This flips it so 0=Dark, 4095=Bright.
    return 4095 - rawValue; 
}

// ---------------- SETUP ----------------

void setupLittleFS() {
    if (!LittleFS.begin()) { LittleFS.format(); LittleFS.begin(); }
    fs_is_ready = true;
}

void initializeWiFi() {
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) delay(500);
}

void reconnectMqtt() {
    while (!client.connected()) {
        String clientId = "PICO_Worker-" + String(random(0xffff), HEX);
        if (client.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
            client.subscribe(mqtt_command_topic);
        } else {
            delay(2000);
        }
    }
}

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
    }
    return false;
}

bool sendWithQoS(const String &payload, unsigned long current_seq) {
    if (WiFi.status() != WL_CONNECTED) return false;
    int retry = 0;
    while (retry < MAX_RETRIES) {
        udp.beginPacket(udp_server_ip, udp_port);
        udp.print(payload);
        udp.endPacket();
        if (waitForAck(current_seq)) return true;
        retry++;
        delay(100 * retry); 
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
    File f = LittleFS.open("/telemetry_log.txt", "r");
    if (!f) return;
    f.close();
    LittleFS.remove("/telemetry_log.txt"); 
}

void handleSensorDelivery(SensorData &data) {
    StaticJsonDocument<256> doc;
    doc["id"] = DEVICE_ID;
    doc["type"] = "WeatherObserved";
    doc["temperature"] = data.tempReading;
    doc["light"] = data.lightReading;
    doc["dateObserved"] = millis();
    doc["qos"] = 1;
    doc["seq"] = seq;

    String payload;
    serializeJson(doc, payload);

    bool delivered = sendWithQoS(payload, seq);
    if (client.connected()) client.publish("/comcs/g04/sensor", payload.c_str(), true);

    if (!delivered) logDataToFile(payload);
    else transmitStoredData();
    seq++;
}

void setup() {
    Serial.begin(115200);
    analogReadResolution(12); // Use full 12-bit resolution (0-4095)
    setupLittleFS();
    initializeWiFi();
    
    dht.begin();
    
    picoClient.setInsecure();
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callback);
    udp.begin(udp_port);
}

void loop() {
    unsigned long now = millis();
    if (!client.connected()) reconnectMqtt();
    client.loop();

    // 2. Sensor Reading
    if (now - lastSensorTime >= 1000) {
        lastSensorTime = now;
        SensorData r;
        
        r.tempReading = dht.readTemperature();
        r.lightReading = readLDR(); // Reading real value from GP26
        
        // Safety check for DHT
        if (isnan(r.tempReading)) {
            r.tempReading = 0.0;
        } else {
             // Optional debug print
             Serial.print("T: "); Serial.print(r.tempReading);
             Serial.print(" | L: "); Serial.println(r.lightReading);
        }
        
        sensorQueue.push_back(r);
    }

    // 3. Network Sending (UDP/MQTT)
    if (now - lastNetworkTime >= current_delay) {
        lastNetworkTime = now;
        while (!sensorQueue.empty()) {
            SensorData data = sensorQueue.front();
            sensorQueue.erase(sensorQueue.begin());
            handleSensorDelivery(data);
        }
    }
}