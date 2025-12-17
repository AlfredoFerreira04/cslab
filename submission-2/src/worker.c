// Raspberry Pi Pico W - Arduino IDE compatible version
// Guaranteed delivery via UDP + MQTT publishing
// No FreeRTOS, uses millis() and simple queue logic

#include <WiFi.h>
#include <WiFiUdp.h>
#include <DHT.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <vector>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>

// ---------------- CONFIG ----------------
const char *ssid = "Pixel_Alf";
const char *password = "alfredopassword04";

const IPAddress udp_server_ip(10, 199, 121, 150);
const int udp_port = 5005;

const char *mqtt_server = "4979254f05ea480283d67c6f0d9f7525.s1.eu.hivemq.cloud";
const char *mqtt_username = "web_client";
const char *mqtt_password = "Password1";
const int mqtt_port = 8883;

#define DHTPIN 4
#define DHTTYPE DHT11

#define MAX_RETRIES 5
#define INITIAL_BACKOFF_MS 200
const char *log_filepath = "/telemetry_log.txt";
const char *DEVICE_ID = "PICO_Device_01";

#define WIFI_CONNECT_TIMEOUT_MS 20000

#define BASE_DELAY_MS 5000
#define MAX_DELAY_MS 60000
#define THROTTLING_THRESHOLD 10
#define THROTTLING_FACTOR 2000

// ---------------- GLOBALS ----------------
DHT dht(DHTPIN, DHTTYPE);
WiFiUDP udp;
WiFiClientSecure picoClient;
PubSubClient client(picoClient);

struct SensorData {
    float tempReading;
    float humReading;
    float lightReading;
};

std::vector<SensorData> sensorQueue;
unsigned long seq = 0;
unsigned long current_delay = BASE_DELAY_MS;
bool fs_is_ready = false;

// ---------------- FORWARD DECLARATIONS ----------------
void logDataToFile(const String &payload);
void transmitStoredData();
void reconnectMqtt();
void setupLittleFS();
void initializeWiFi();
bool waitForAck(unsigned long mySeq, const char *myId);
bool sendWithQoS(const String &payload, unsigned long current_seq);
void publishMessage(const char *topic, const String &payload, bool retained);
bool handleSensorDelivery(SensorData &data);

// ---------------- FUNCTIONS ----------------

void setupLittleFS() {
    if (!LittleFS.begin()) {
        Serial.println("LittleFS Mount Failed! Formatting...");
        if (LittleFS.format() && LittleFS.begin()) {
            Serial.println("Mounted after format.");
            fs_is_ready = true;
        } else {
            Serial.println("FATAL: LittleFS not available.");
        }
    } else {
        Serial.println("LittleFS mounted successfully.");
        fs_is_ready = true;
    }
}

void initializeWiFi() {
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");

    unsigned long start_time = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start_time < WIFI_CONNECT_TIMEOUT_MS) {
        Serial.print(".");
        delay(500);
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nConnected! IP: " + WiFi.localIP().toString());
    } else {
        Serial.println("\nERROR: Failed to connect to WiFi within timeout!");
    }
}

void reconnectMqtt() {
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection... ");
        String clientId = "PICO_W-" + String(random(0xffff), HEX);

        if (client.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
            Serial.println("connected");
            client.subscribe("/comcs/g04/commands");
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" retrying in 5 seconds");
            delay(5000);
        }
    }
}

void callback(char *topic, byte *payload, unsigned int length) {
    Serial.print("Message received: ");
    for (int i = 0; i < length; i++) Serial.print((char)payload[i]);
    Serial.println();
}

void publishMessage(const char *topic, const String &payload, bool retained) {
    if (client.publish(topic, payload.c_str(), retained)) {
        Serial.println("JSON published to " + String(topic));
    } else {
        Serial.print("MQTT publish failed for topic: ");
        Serial.println(topic);
    }
}

bool waitForAck(unsigned long mySeq, const char *myId) {
    unsigned long start = millis();
    char incoming[512];

    while (millis() - start < 800) {
        int packetSize = udp.parsePacket();
        if (packetSize) {
            int len = udp.read(incoming, sizeof(incoming) - 1);
            incoming[len] = 0;

            StaticJsonDocument<128> doc;
            DeserializationError error = deserializeJson(doc, incoming);
            if (error) continue;

            const char *type = doc["type"] | "";
            const char *id = doc["id"] | "";
            unsigned long seq = doc["seq"] | 0;

            if (strcmp(type, "ACK") == 0 && strcmp(id, myId) == 0 && seq == mySeq) {
                Serial.println("ACK received!");
                return true;
            }
        }
    }
    return false;
}

bool sendWithQoS(const String &payload, unsigned long current_seq) {
    bool delivered = false;
    int retry_count = 0;
    unsigned long backoff_ms = INITIAL_BACKOFF_MS;

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi disconnected, cannot send.");
        return false;
    }

    do {
        udp.beginPacket(udp_server_ip, udp_port);
        udp.print(payload);
        udp.endPacket();

        Serial.print("Sent UDP (Seq ");
        Serial.print(current_seq);
        Serial.print("): ");
        Serial.println(payload.substring(0, min((int)payload.length(), 60)) + "...");

        delivered = waitForAck(current_seq, DEVICE_ID);

        if (!delivered) {
            retry_count++;
            Serial.print("No ACK -> retrying #");
            Serial.println(retry_count);

            if (retry_count >= MAX_RETRIES) {
                Serial.println("ERROR: Max retries reached. Logging to file.");
                return false;
            }

            delay(backoff_ms);
            backoff_ms = min(backoff_ms * 2, 5000ul);
        }
    } while (!delivered);

    return delivered;
}

void logDataToFile(const String &payload) {
    if (!fs_is_ready) return;

    File file = LittleFS.open(log_filepath, "a");
    if (!file) return;
    file.println(payload);
    file.close();
}

void transmitStoredData() {
    if (!fs_is_ready) return;

    File file = LittleFS.open(log_filepath, "r");
    if (!file) return;

    std::vector<String> failed_messages;
    while (file.available()) {
        String line = file.readStringUntil('\n');
        line.trim();
        if (line.length() == 0) continue;

        unsigned long stored_seq = 0;
        StaticJsonDocument<128> doc;
        if (deserializeJson(doc, line)) continue;
        stored_seq = doc["seq"] | 0;

        if (!sendWithQoS(line, stored_seq)) failed_messages.push_back(line);
    }
    file.close();

    if (failed_messages.empty()) LittleFS.remove(log_filepath);
    else {
        File f = LittleFS.open(log_filepath, "w");
        for (const String &msg : failed_messages) f.println(msg);
        f.close();
    }
}

bool handleSensorDelivery(SensorData &data) {
    StaticJsonDocument<256> doc;
    doc["id"] = DEVICE_ID;
    doc["type"] = "WeatherObserved";
    doc["temperature"] = data.tempReading;
    doc["relativeHumidity"] = data.humReading;
    doc["dateObserved"] = millis();
    doc["status"] = "OPERATIONAL";
    doc["qos"] = 1;
    doc["seq"] = seq;

    String payload;
    serializeJson(doc, payload);

    bool delivered = sendWithQoS(payload, seq);
    publishMessage("/comcs/g04/sensor", payload, true);

    if (!delivered) logDataToFile(payload);
    else transmitStoredData();

    seq++;
    return delivered;
}

// ---------------- LOOP-BASED "TASKS" ----------------
unsigned long lastSensorTime = 0;
unsigned long lastNetworkTime = 0;

void setup() {
    Serial.begin(9600);
    Serial.println("Hello Pico W!");

    setupLittleFS();
    initializeWiFi();

    picoClient.setInsecure();
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callback);

    udp.begin(udp_port);
    dht.begin();
}

void loop() {
    unsigned long now = millis();

    // SENSOR TASK - every 1 second
    if (now - lastSensorTime >= 1000) {
        lastSensorTime = now;

        SensorData reading;
        reading.tempReading = dht.readTemperature();
        reading.humReading = dht.readHumidity();
        reading.lightReading = 0;

        if (isnan(reading.tempReading) || isnan(reading.humReading)) {
            reading.tempReading = 0.0;
            reading.humReading = 0.0;
        }

        sensorQueue.push_back(reading);
    }

    // NETWORK TASK - every current_delay
    if (now - lastNetworkTime >= current_delay) {
        lastNetworkTime = now;

        // Handle WiFi
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("WiFi lost! Reconnecting...");
            initializeWiFi();
        }

        // Handle MQTT
        if (!client.connected()) reconnectMqtt();
        client.loop();

        // Send queued sensor data
        while (!sensorQueue.empty()) {
            SensorData data = sensorQueue.front();
            sensorQueue.erase(sensorQueue.begin());
            handleSensorDelivery(data);
        }

        // Adaptive throttling (simple)
        int backlog = sensorQueue.size();
        if (backlog > THROTTLING_THRESHOLD) {
            current_delay = min(BASE_DELAY_MS + (backlog - THROTTLING_THRESHOLD) * THROTTLING_FACTOR, MAX_DELAY_MS);
        } else {
            current_delay = BASE_DELAY_MS;
        }
    }
}
