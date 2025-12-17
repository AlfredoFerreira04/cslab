// alert_udp_server.c
// Compile: gcc alert_udp_server.c -o alert_udp_server -lcjson -lpaho-mqtt3c -lpthread
// Run: ./alert_udp_server
//
// Requires: cJSON, Paho MQTT C client, and pthreads library.
#include "asm.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <time.h>
#include <errno.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <math.h>        // For fabs() function used in differential calculation
#include <cjson/cJSON.h> // For JSON parsing (Smartdata model)
#include <MQTTClient.h>  // Paho MQTT C client
#include <pthread.h>     // For creating monitoring thread
#include <unistd.h>      // For usleep

// Network Configuration (Req 2a)
#define PORT 5005
#define BUFFER_SIZE 8192
#define MAX_DEVICES 1024
#define ALERT_LOGFILE "alerts.log" // File for logging critical events (Req 2d)

// Equipment valid ranges (for basic data validation)
#define TEMP_MIN 0.0
#define TEMP_MAX 50.0
#define HUM_MIN 20.0
#define HUM_MAX 80.0

// Alert thresholds
#define TEMP_DIFF_THRESHOLD 3.0 // degrees
#define HUM_DIFF_THRESHOLD 20.0 // percent

// NEW: Inactivity Timeout Configuration
#define INACTIVITY_TIMEOUT_SEC 10 // Client is considered dead after 60 seconds of no reports
#define MONITOR_INTERVAL_SEC 5    // Check every 10 seconds

// MQTT Configuration
#define MQTT_ADDRESS "ssl://4979254f05ea480283d67c6f0d9f7525.s1.eu.hivemq.cloud:8883"
#define MQTT_CLIENT_ID "udp_alert_server"
#define MQTT_ALERT_TOPIC "/cslab/g01/alerts"

// HiveMQ Credentials
#define MQTT_USERNAME "web_client"
#define MQTT_PASSWORD "Password1"

#define MQTT_PREF_LIGHT_TOPIC "/cslab/g01/preferences/light"
#define MQTT_PREF_TEMP_TOPIC "/cslab/g01/preferences/temperature"

MQTTClient client;

// Structure to track the state of each sending device (Req 2c)
typedef struct
{
    char id[128];
    double temperature;
    double humidity;
    double light;          // Added: To match SensorData struct
    char status[32];       // Added: To match "OPERATIONAL" payload
    char dateObserved[64]; // Updated: Handles string or millis()
    struct sockaddr_in addr;
    int has_seq;
    long last_seq;
    time_t last_seen;
} device_t;

// Global storage for tracking connected devices (Req 2c)
static device_t devices[MAX_DEVICES];
static int device_count = 0;
static FILE *alert_log = NULL;

// Helper function definitions
static void log_alert(const char *message);

static device_t *find_device_by_id(const char *id);
static device_t *add_or_get_device(const char *id, struct sockaddr_in *addr);
static void send_ack(int sockfd, struct sockaddr_in *client_addr, socklen_t addrlen, const char *id, long seq);

static void log_alert(const char *message)
{
    // print timestamped message to stdout and file
    time_t now = time(NULL);
    struct tm tm_now;
    localtime_r(&now, &tm_now);
    char timebuf[64];
    strftime(timebuf, sizeof(timebuf), "%Y-%m-%d %H:%M:%S", &tm_now);

    printf("[%s] %s\n", timebuf, message);

    if (alert_log)
    {
        fprintf(alert_log, "[%s] %s\n", timebuf, message);
        fflush(alert_log);
    }
}

// Function running in a separate thread to keep the MQTT connection alive.
void *mqtt_thread_func(void *arg)
{
    while (1)
    {
        // Keep MQTT alive
        MQTTClient_yield();
        usleep(100 * 1000); // 100 ms sleep to avoid busy loop
    }
    return NULL;
}

// Function to log alerts to stdout and a file (Req 2d)
// and send alert via MQTT (Req 2f)
static void log_alert_dual(const char *device,
                           const char *alert_type,
                           const char *message)
{
    /* --------- 1) PRINT & SAVE LOG ENTRY --------- */
    char formatted[512]; // Increased size for complex messages
    snprintf(formatted, sizeof(formatted),
             "%s: device=%s: %s",
             alert_type, device, message);

    log_alert(formatted); // Now calls the standalone log_alert function

    /* --------- 2) BUILD JSON ALERT FOR MQTT --------- */
    cJSON *root = cJSON_CreateObject();
    if (!root)
        return;

    // Add current time (ISO 8601 format)
    time_t now = time(NULL);
    struct tm tm_now;
    localtime_r(&now, &tm_now);
    char timebuf[64];
    strftime(timebuf, sizeof(timebuf), "%Y-%m-%dT%H:%M:%S", &tm_now);
    cJSON_AddStringToObject(root, "timestamp", timebuf);

    cJSON_AddStringToObject(root, "device", device);
    cJSON_AddStringToObject(root, "alertType", alert_type);
    cJSON_AddStringToObject(root, "message", message);

    char *json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    if (!json_str)
        return;

    /* --------- 3) PUBLISH JSON TO MQTT --------- */

    MQTTClient_message msg = MQTTClient_message_initializer;
    MQTTClient_deliveryToken token;

    msg.payload = json_str;
    msg.payloadlen = (int)strlen(json_str);
    msg.qos = 1; // Guaranteed delivery via MQTT
    msg.retained = 0;

    int rc = MQTTClient_publishMessage(client, MQTT_ALERT_TOPIC, &msg, &token);
    if (rc == MQTTCLIENT_SUCCESS)
    {
        // Wait briefly for confirmation
        MQTTClient_waitForCompletion(client, token, 1000);
    }
    else
    {
        // Keep silent to avoid log spam, as the failure might be transient.
    }

    free(json_str);
}

// Function running in a separate thread to check for client inactivity (NEW REQUIREMENT)
void *monitor_device_status(void *arg)
{
    printf("Device monitoring thread started. Timeout: %d sec.\n", INACTIVITY_TIMEOUT_SEC);
    time_t current_time;
    char message[256];

    while (1)
    {
        sleep(MONITOR_INTERVAL_SEC); // Check devices periodically

        current_time = time(NULL);

        for (int i = 0; i < device_count; ++i)
        {
            device_t *dev = &devices[i];
            double inactivity_duration = difftime(current_time, dev->last_seen);

            if (inactivity_duration > INACTIVITY_TIMEOUT_SEC)
            {
                // Trigger an alert for client inactivity
                snprintf(message, sizeof(message),
                         "Client has not reported in %.0f seconds. Suspected failure.",
                         inactivity_duration);

                log_alert_dual(dev->id, "CLIENT_INACTIVITY", message);

                // Optional: To prevent immediate spamming of the same alert,
                // you might want to increase dev->last_seen temporarily or
                // use a separate flag. For simplicity, we just log the alert.
            }
        }
    }
    return NULL;
}

// Looks up a device based on its unique ID
static device_t *find_device_by_id(const char *id)
{
    for (int i = 0; i < device_count; ++i)
    {
        if (strcmp(devices[i].id, id) == 0)
            return &devices[i];
    }
    return NULL;
}

// Looks up a device based on its network address (less reliable, but available)
static device_t *find_device_by_addr(struct sockaddr_in *addr)
{
    for (int i = 0; i < device_count; ++i)
    {
        if (devices[i].addr.sin_addr.s_addr == addr->sin_addr.s_addr &&
            devices[i].addr.sin_port == addr->sin_port)
            return &devices[i];
    }
    return NULL;
}

// Adds a new device or retrieves an existing one (Req 2c)
static device_t *add_or_get_device(const char *id, struct sockaddr_in *addr)
{
    device_t *d = find_device_by_id(id);
    if (d)
    {
        // If device exists, update its network address and last seen time
        d->addr = *addr;
        d->last_seen = time(NULL);
        return d;
    }

    // Add new device if space is available
    if (device_count >= MAX_DEVICES)
        return NULL;
    d = &devices[device_count++];

    // Initialize new device struct
    strncpy(d->id, id, sizeof(d->id) - 1);
    d->id[sizeof(d->id) - 1] = '\0';
    d->temperature = 0;
    d->humidity = 0;
    d->dateObserved[0] = '\0';
    d->addr = *addr;
    d->has_seq = 0;
    d->last_seq = -1;
    d->last_seen = time(NULL);
    return d;
}

// Sends an ACK message back to the client for QoS=1 (Guaranteed Delivery) (Req 2b)
static void send_ack(int sockfd, struct sockaddr_in *client_addr, socklen_t addrlen, const char *id, long seq)
{
    if (!id)
        return;

    // Use cJSON to build the ACK response
    cJSON *ack = cJSON_CreateObject();
    if (!ack)
    {
        perror("cJSON_CreateObject failed in send_ack");
        return;
    }

    cJSON_AddStringToObject(ack, "type", "ACK");
    cJSON_AddStringToObject(ack, "id", id);
    cJSON_AddNumberToObject(ack, "seq", (double)seq); // Sequence number must match the received one

    char *out = cJSON_PrintUnformatted(ack); // Print compact JSON string
    if (out)
    {
        // Send the ACK via the UDP socket
        ssize_t sent = sendto(sockfd, out, strlen(out), 0, (struct sockaddr *)client_addr, addrlen);
        if (sent < 0)
        {
            perror("sendto (ACK) failed");
        }
        free(out); // Free cJSON string
    }
    else
    {
        perror("cJSON_PrintUnformatted failed in send_ack");
    }
    cJSON_Delete(ack); // Free cJSON object
}

int mqtt_message_arrived(void *context,
                         char *topicName,
                         int topicLen,
                         MQTTClient_message *message)
{
    char logbuf[512];

    /* Ensure payload is null-terminated */
    char payload[256];
    int len = (message->payloadlen < 255) ? message->payloadlen : 255;
    memcpy(payload, message->payload, len);
    payload[len] = '\0';

    if (strcmp(topicName, "/cslab/g01/preferences/temperature") == 0)
    {
        snprintf(logbuf, sizeof(logbuf),
                 "Temperature preference updated to %s",
                 payload);
    }
    else if (strcmp(topicName, "/cslab/g01/preferences/light") == 0)
    {
        snprintf(logbuf, sizeof(logbuf),
                 "Light preference updated to %s",
                 payload);
    }
    else
    {
        /* Fallback for unexpected topics */
        snprintf(logbuf, sizeof(logbuf),
                 "unknown: \"Preference update on %s with value %s\"",
                 topicName, payload);
    }

    log_alert(logbuf);

    MQTTClient_freeMessage(&message);
    MQTTClient_free(topicName);

    return 1;
}

int main()
{
    int sockfd;
    struct sockaddr_in server_addr, client_addr;
    char buffer[BUFFER_SIZE];
    char client_ip_str[INET_ADDRSTRLEN];
    char log_message[BUFFER_SIZE + 256];
    pthread_t mqtt_thread;
    pthread_t monitor_thread; // NEW: Monitoring thread ID
    int mqtt_thread_created = 0;
    int monitor_thread_created = 0;

    // Open the alert log file for appending
    alert_log = fopen(ALERT_LOGFILE, "a");
    if (!alert_log)
    {
        perror("Failed to open alert log file");
    }

    // Create UDP socket (AF_INET for IPv4, SOCK_DGRAM for UDP)
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        perror("Failed to create socket");
        exit(EXIT_FAILURE);
    }

    // Configure server address structure
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY; // Listen on all interfaces
    server_addr.sin_port = htons(PORT);       // Convert port to network byte order

    // Bind server socket to the address and port (Req 2a)
    if (bind(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    {
        perror("Bind failed");
        close(sockfd);
        exit(EXIT_FAILURE);
    }

    printf("Alert UDP server running on port %d...\n", PORT);

    // --- MQTT Initialization ---
    MQTTClient_create(&client, MQTT_ADDRESS, MQTT_CLIENT_ID, MQTTCLIENT_PERSISTENCE_NONE, NULL);

    MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
    MQTTClient_SSLOptions ssl_opts = MQTTClient_SSLOptions_initializer;
    MQTTClient_setCallbacks(client, NULL, NULL, mqtt_message_arrived, NULL);
    // NOTE: For HiveMQ/public brokers, often trustStore is not needed if running on a modern OS with root certificates.
    // However, including the option is necessary for secure connection setup.
    ssl_opts.enableServerCertAuth = 1;
    // ssl_opts.trustStore = "./cert.pem"; // Commented out, but required if cert file is used.

    conn_opts.keepAliveInterval = 20;
    conn_opts.cleansession = 1;
    conn_opts.username = MQTT_USERNAME;
    conn_opts.password = MQTT_PASSWORD;
    conn_opts.ssl = &ssl_opts;

    int rc;
    if ((rc = MQTTClient_connect(client, &conn_opts)) != MQTTCLIENT_SUCCESS)
    {
        printf("Failed to connect to MQTT, return code %d\n", rc);
        // Do not exit, continue to serve UDP telemetry
    }
    else
    {
        printf("Connected to MQTT broker at %s\n", MQTT_ADDRESS);
        MQTTClient_subscribe(client, MQTT_PREF_LIGHT_TOPIC, 1);
        MQTTClient_subscribe(client, MQTT_PREF_TEMP_TOPIC, 1);

        printf("Subscribed to preference topics:\n");
        printf(" - %s\n", MQTT_PREF_LIGHT_TOPIC);
        printf(" - %s\n", MQTT_PREF_TEMP_TOPIC);

        // Start the thread to keep the MQTT connection alive
        if (pthread_create(&mqtt_thread, NULL, mqtt_thread_func, NULL) != 0)
        {
            perror("Failed to create MQTT thread");
        }
        else
        {
            mqtt_thread_created = 1;
        }
    }

    // --- Device Monitoring Initialization ---
    if (pthread_create(&monitor_thread, NULL, monitor_device_status, NULL) != 0)
    {
        perror("Failed to create device monitor thread");
    }
    else
    {
        monitor_thread_created = 1;
    }

    // Main server loop
    while (1)
    {
        socklen_t len = sizeof(client_addr);

        ssize_t n = recvfrom(sockfd, buffer, BUFFER_SIZE - 1, 0,
                             (struct sockaddr *)&client_addr, &len);

        if (n < 0)
        {
            if (errno == EINTR)
                continue;
            perror("Error receiving data");
            continue;
        }

        buffer[n] = '\0'; // Null-terminate

        if (inet_ntop(AF_INET, &(client_addr.sin_addr), client_ip_str, INET_ADDRSTRLEN) == NULL)
        {
            strcpy(client_ip_str, "UNKNOWN_IP");
        }

        // --- 1. PARSE JSON ---
        cJSON *root = cJSON_Parse(buffer);
        if (!root)
        {
            snprintf(log_message, sizeof(log_message), "Invalid JSON from %s", client_ip_str);
            log_alert(log_message);
            cJSON_Delete(root);
            continue;
        }

        // --- 2. EXTRACT FIELDS (Updated for new Payload) ---
        cJSON *jid = cJSON_GetObjectItemCaseSensitive(root, "id");
        cJSON *jtype = cJSON_GetObjectItemCaseSensitive(root, "type");     // NEW
        cJSON *jstatus = cJSON_GetObjectItemCaseSensitive(root, "status"); // NEW
        cJSON *jtemp = cJSON_GetObjectItemCaseSensitive(root, "temperature");
        cJSON *jhum = cJSON_GetObjectItemCaseSensitive(root, "relativeHumidity");
        cJSON *jlight = cJSON_GetObjectItemCaseSensitive(root, "light"); // Optional
        cJSON *jdate = cJSON_GetObjectItemCaseSensitive(root, "dateObserved");
        cJSON *jseq = cJSON_GetObjectItemCaseSensitive(root, "seq");
        cJSON *jqos = cJSON_GetObjectItemCaseSensitive(root, "qos");

        // Validate Mandatory Fields
        if (!cJSON_IsString(jid) || !cJSON_IsNumber(jtemp) || !cJSON_IsNumber(jhum))
        {
            snprintf(log_message, sizeof(log_message), "Missing mandatory fields from %s", client_ip_str);
            log_alert(log_message);
            cJSON_Delete(root);
            continue;
        }

        // Validate Type (Optional but good practice)
        if (cJSON_IsString(jtype) && strcmp(jtype->valuestring, "WeatherObserved") != 0)
        {
            // Just a warning, proceed anyway
            printf("Warning: Received unknown type '%s'\n", jtype->valuestring);
        }

        // --- 3. EXTRACT VALUES ---
        const char *id = jid->valuestring;
        double temp = jtemp->valuedouble;
        double hum = jhum->valuedouble;
        double light = cJSON_IsNumber(jlight) ? jlight->valuedouble : -1.0; // Default -1 if missing

        // Handle Status
        char status_buf[32] = "UNKNOWN";
        if (cJSON_IsString(jstatus))
        {
            strncpy(status_buf, jstatus->valuestring, sizeof(status_buf) - 1);
            status_buf[sizeof(status_buf) - 1] = '\0';
        }

        // Handle Date (CRITICAL FIX: Pico sends millis() as Number, not String)
        char date_buf[64] = "0";
        if (cJSON_IsNumber(jdate))
        {
            // Convert millis() number to string
            snprintf(date_buf, sizeof(date_buf), "%lu", (unsigned long)jdate->valuedouble);
        }
        else if (cJSON_IsString(jdate))
        {
            strncpy(date_buf, jdate->valuestring, sizeof(date_buf) - 1);
            date_buf[sizeof(date_buf) - 1] = '\0';
        }

        long seq = -1;
        int qos = 0;

        if (cJSON_IsNumber(jseq))
            seq = (long)cJSON_GetNumberValue(jseq);
        if (cJSON_IsNumber(jqos))
            qos = jqos->valueint;

        // --- 4. DEVICE MANAGEMENT ---
        device_t *dev = add_or_get_device(id, &client_addr);
        if (!dev)
        {
            cJSON_Delete(root);
            continue;
        }

        // QoS Logic (Deduplication)
        if (qos == 1)
        {
            if (seq == -1)
            {
                cJSON_Delete(root);
                continue;
            }
            if (dev->has_seq && seq == dev->last_seq)
            {
                send_ack(sockfd, &client_addr, len, id, seq);
                cJSON_Delete(root);
                continue;
            }
        }

        // Update Device State
        dev->temperature = temp;
        dev->humidity = hum;
        dev->light = light;                                        // Store light
        strncpy(dev->status, status_buf, sizeof(dev->status) - 1); // Store status
        strncpy(dev->dateObserved, date_buf, sizeof(dev->dateObserved) - 1);
        dev->last_seen = time(NULL);

        if (qos == 1)
        {
            dev->has_seq = 1;
            dev->last_seq = seq;
            send_ack(sockfd, &client_addr, len, id, seq);
        }

        // Print Status Update
        printf("[%s] %s (Status: %s) -> T:%.2f L:%.2f [Seq: %ld]\n",
               client_ip_str, id, status_buf, temp, light, seq);

        // --- 5. ALERTS (Logic remains same, just updated logging) ---
        if (temp < TEMP_MIN || temp > TEMP_MAX)
        {
            snprintf(log_message, sizeof(log_message), "Temp %.2f out of range", temp);
            log_alert_dual(id, "TEMP_ALERT", log_message);
        }

        // Check differential
        for (int i = 0; i < device_count; ++i)
        {
            device_t *other = &devices[i];
            if (strcmp(other->id, dev->id) == 0)
                continue;

            // THIS COMMENTED BLOCK TO BE REPLACED BY ASM86
            int result = check_limit(dev->temperature, other->temperature, TEMP_DIFF_THRESHOLD);

            if (result == 1)
            {
                snprintf(log_message, sizeof(log_message), "Compared with %s, there is a differential that is too great (thresholds: %+0.2f°C / %+0.2f%%, respectively).",
                         other->id, TEMP_DIFF_THRESHOLD, HUM_DIFF_THRESHOLD);
                log_alert_dual(id, "DIFFERENTIAL_ALERT", log_message);
            }
        }

        cJSON_Delete(root);
    }

    // --- Cleanup and Exit ---
    if (monitor_thread_created)
    {
        pthread_cancel(monitor_thread);
        pthread_join(monitor_thread, NULL);
    }
    if (mqtt_thread_created)
    {
        pthread_cancel(mqtt_thread);
        pthread_join(mqtt_thread, NULL);
    }

    if (MQTTClient_isConnected(client))
        MQTTClient_disconnect(client, 1000);
    MQTTClient_destroy(&client);

    if (alert_log)
        fclose(alert_log);
    close(sockfd);
    return 0;
}
