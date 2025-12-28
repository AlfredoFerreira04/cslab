// alert_udp_server.c
// Compile: gcc alert_udp_server.c -o alert_udp_server -lcjson -lpaho-mqtt3c -lpthread
// Run: ./alert_udp_server

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
#include <math.h> 
#include <cjson/cJSON.h> 
#include <MQTTClient.h> 
#include <pthread.h>     
#include <unistd.h>      

// --- CONFIGURATION ---
#define PORT 5005
#define BUFFER_SIZE 8192
#define MAX_DEVICES 1024
#define ALERT_LOGFILE "alerts.log"

#define TEMP_MIN_VALID 0.0
#define TEMP_MAX_VALID 50.0

// PREFERENCES
#define DEFAULT_PREF_TEMP_MIN 18.0
#define DEFAULT_PREF_TEMP_MAX 26.0
#define DEFAULT_PREF_LIGHT_MIN 30.0 

#define INACTIVITY_TIMEOUT_SEC 15 // Increased to be more forgiving
#define MONITOR_INTERVAL_SEC 5

// MQTT Config
#define MQTT_ADDRESS "ssl://4979254f05ea480283d67c6f0d9f7525.s1.eu.hivemq.cloud:8883"
#define MQTT_CLIENT_ID "udp_alert_server"
#define MQTT_ALERT_TOPIC "/cslab/g01/alerts"
#define MQTT_COMMAND_TOPIC "/cslab/g01/commands"

#define MQTT_USERNAME "web_client"
#define MQTT_PASSWORD "Password1"

#define MQTT_PREF_LIGHT_TOPIC "/cslab/g01/preferences/light"
#define MQTT_PREF_TEMP_TOPIC "/cslab/g01/preferences/temperature"

MQTTClient client;

typedef struct
{
    char id[128];
    double temperature;
    double light;          
    char status[32];       
    char dateObserved[64]; 
    struct sockaddr_in addr;
    int has_seq;
    long last_seq;
    time_t last_seen;
} device_t;

static device_t devices[MAX_DEVICES];
static int device_count = 0;
static FILE *alert_log = NULL;

char designated_actuator_id[128] = ""; 
double pref_temp_min = DEFAULT_PREF_TEMP_MIN;
double pref_temp_max = DEFAULT_PREF_TEMP_MAX;
double pref_light_min = DEFAULT_PREF_LIGHT_MIN;

// --- NEW: STATE TRACKING TO PREVENT SPAM ---
int last_sent_heat_state = -1; // -1: Unknown, 0: OFF, 1: ON
int last_sent_light_state = -1; // -1: Unknown, 0: OFF, 1: ON

static void log_alert(const char *message);
static device_t *add_or_get_device(const char *id, struct sockaddr_in *addr);
static void send_ack(int sockfd, struct sockaddr_in *client_addr, socklen_t addrlen, const char *id, long seq);
void send_actuator_command(const char *type, const char *action);

static void log_alert(const char *message)
{
    time_t now = time(NULL);
    struct tm tm_now;
    localtime_r(&now, &tm_now);
    char timebuf[64];
    strftime(timebuf, sizeof(timebuf), "%Y-%m-%d %H:%M:%S", &tm_now);
    printf("[%s] %s\n", timebuf, message);
    if (alert_log) {
        fprintf(alert_log, "[%s] %s\n", timebuf, message);
        fflush(alert_log);
    }
}

void *mqtt_thread_func(void *arg)
{
    while (1) {
        MQTTClient_yield();
        usleep(100 * 1000); 
    }
    return NULL;
}

void select_actuator() {
    if (device_count > 0 && strlen(designated_actuator_id) == 0) {
        strncpy(designated_actuator_id, devices[0].id, sizeof(designated_actuator_id)-1);
        char msg[256];
        snprintf(msg, sizeof(msg), "SYSTEM: Designated %s as the primary ACTUATOR.", designated_actuator_id);
        log_alert(msg);
        // Reset states on new actuator
        last_sent_heat_state = -1;
        last_sent_light_state = -1;
    }
}

void send_actuator_command(const char *type, const char *action) {
    if (strlen(designated_actuator_id) == 0) return;

    // --- LOGIC CHECK: Don't send if state matches last sent ---
    int desired_state = (strcmp(action, "ON") == 0) ? 1 : 0;
    
    if (strcmp(type, "HEAT") == 0) {
        if (last_sent_heat_state == desired_state) return; // No change needed
        last_sent_heat_state = desired_state;
    } 
    else if (strcmp(type, "LIGHT") == 0) {
        if (last_sent_light_state == desired_state) return; // No change needed
        last_sent_light_state = desired_state;
    }

    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "target_id", designated_actuator_id);
    cJSON_AddStringToObject(root, "type", type);
    cJSON_AddStringToObject(root, "action", action);

    char *json_str = cJSON_PrintUnformatted(root);
    
    MQTTClient_message msg = MQTTClient_message_initializer;
    MQTTClient_deliveryToken token;

    msg.payload = json_str;
    msg.payloadlen = (int)strlen(json_str);
    msg.qos = 1;
    msg.retained = 0;

    MQTTClient_publishMessage(client, MQTT_COMMAND_TOPIC, &msg, &token);
    
    char logbuf[256];
    snprintf(logbuf, sizeof(logbuf), "ORDER SENT: Told %s to turn %s %s", designated_actuator_id, type, action);
    log_alert(logbuf);

    free(json_str);
    cJSON_Delete(root);
}

void assess_environment() {
    if (device_count == 0) return;

    double sum_temp = 0;
    double sum_light = 0;
    int valid_count = 0;

    for (int i = 0; i < device_count; i++) {
        // Only count valid readings to avoid skewing averages with -10.0 or 0.0 errors
        if (devices[i].temperature > TEMP_MIN_VALID && devices[i].temperature < TEMP_MAX_VALID) {
             sum_temp += devices[i].temperature;
             sum_light += devices[i].light;
             valid_count++;
        }
    }

    if (valid_count == 0) return;

    double avg_temp = sum_temp / valid_count;
    double avg_light = sum_light / valid_count;

    // ACTUATE HEAT
    if (avg_temp < pref_temp_min) send_actuator_command("HEAT", "ON");
    else if (avg_temp > pref_temp_max) send_actuator_command("HEAT", "OFF");

    // ACTUATE LIGHT
    if (avg_light < pref_light_min) send_actuator_command("LIGHT", "ON");
    else send_actuator_command("LIGHT", "OFF");
}

static void log_alert_dual(const char *device, const char *alert_type, const char *message)
{
    char formatted[512];
    snprintf(formatted, sizeof(formatted), "%s: device=%s: %s", alert_type, device, message);
    log_alert(formatted);
}

void *monitor_device_status(void *arg)
{
    time_t current_time;
    char message[256];
    while (1) {
        sleep(MONITOR_INTERVAL_SEC);
        current_time = time(NULL);
        for (int i = 0; i < device_count; ++i) {
            double duration = difftime(current_time, devices[i].last_seen);
            if (duration > INACTIVITY_TIMEOUT_SEC) {
                snprintf(message, sizeof(message), "Client silent for %.0f sec.", duration);
                log_alert_dual(devices[i].id, "CLIENT_INACTIVITY", message);
                
                if (strcmp(devices[i].id, designated_actuator_id) == 0) {
                    log_alert("SYSTEM: Actuator is unresponsive. Resetting designation.");
                    designated_actuator_id[0] = '\0';
                }
            }
        }
    }
    return NULL;
}

static device_t *find_device_by_id(const char *id)
{
    for (int i = 0; i < device_count; ++i) {
        if (strcmp(devices[i].id, id) == 0) return &devices[i];
    }
    return NULL;
}

static device_t *add_or_get_device(const char *id, struct sockaddr_in *addr)
{
    device_t *d = find_device_by_id(id);
    if (d) {
        d->addr = *addr;
        d->last_seen = time(NULL);
        return d;
    }
    if (device_count >= MAX_DEVICES) return NULL;
    d = &devices[device_count++];
    strncpy(d->id, id, sizeof(d->id) - 1);
    d->id[sizeof(d->id) - 1] = '\0';
    d->temperature = 0;
    d->light = 0;
    d->dateObserved[0] = '\0';
    d->addr = *addr;
    d->has_seq = 0;
    d->last_seq = -1;
    d->last_seen = time(NULL);
    
    select_actuator();
    return d;
}

static void send_ack(int sockfd, struct sockaddr_in *client_addr, socklen_t addrlen, const char *id, long seq)
{
    if (!id) return;
    cJSON *ack = cJSON_CreateObject();
    cJSON_AddStringToObject(ack, "type", "ACK");
    cJSON_AddStringToObject(ack, "id", id);
    cJSON_AddNumberToObject(ack, "seq", (double)seq);

    char *out = cJSON_PrintUnformatted(ack);
    if (out) {
        sendto(sockfd, out, strlen(out), 0, (struct sockaddr *)client_addr, addrlen);
        free(out);
    }
    cJSON_Delete(ack);
}

int mqtt_message_arrived(void *context, char *topicName, int topicLen, MQTTClient_message *message)
{
    char payload[256];
    int len = (message->payloadlen < 255) ? message->payloadlen : 255;
    memcpy(payload, message->payload, len);
    payload[len] = '\0';

    if (strcmp(topicName, MQTT_PREF_TEMP_TOPIC) == 0) {
        double min, max;
        if (sscanf(payload, "%lf,%lf", &min, &max) == 2) {
            pref_temp_min = min;
            pref_temp_max = max;
            printf("PREFS: Updated Temp Range: %.1f - %.1f\n", pref_temp_min, pref_temp_max);
        }
    }
    else if (strcmp(topicName, MQTT_PREF_LIGHT_TOPIC) == 0) {
        double min;
        if (sscanf(payload, "%lf", &min) == 1) {
            pref_light_min = min;
            printf("PREFS: Updated Light Min: %.1f\n", pref_light_min);
        }
    }

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
    pthread_t mqtt_thread, monitor_thread;
    
    alert_log = fopen(ALERT_LOGFILE, "a");
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) exit(EXIT_FAILURE);
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(PORT);

    if (bind(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        perror("Bind failed");
        exit(EXIT_FAILURE);
    }

    printf("Server running on port %d. Waiting for workers...\n", PORT);

    MQTTClient_create(&client, MQTT_ADDRESS, MQTT_CLIENT_ID, MQTTCLIENT_PERSISTENCE_NONE, NULL);
    MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
    MQTTClient_SSLOptions ssl_opts = MQTTClient_SSLOptions_initializer;
    MQTTClient_setCallbacks(client, NULL, NULL, mqtt_message_arrived, NULL);
    ssl_opts.enableServerCertAuth = 1;
    conn_opts.keepAliveInterval = 20;
    conn_opts.cleansession = 1;
    conn_opts.username = MQTT_USERNAME;
    conn_opts.password = MQTT_PASSWORD;
    conn_opts.ssl = &ssl_opts;

    if (MQTTClient_connect(client, &conn_opts) == MQTTCLIENT_SUCCESS) {
        printf("Connected to MQTT.\n");
        MQTTClient_subscribe(client, MQTT_PREF_LIGHT_TOPIC, 1);
        MQTTClient_subscribe(client, MQTT_PREF_TEMP_TOPIC, 1);
        pthread_create(&mqtt_thread, NULL, mqtt_thread_func, NULL);
    }

    pthread_create(&monitor_thread, NULL, monitor_device_status, NULL);

    while (1) {
        socklen_t len = sizeof(client_addr);
        ssize_t n = recvfrom(sockfd, buffer, BUFFER_SIZE - 1, 0, (struct sockaddr *)&client_addr, &len);

        if (n < 0) continue;
        buffer[n] = '\0';
        if (inet_ntop(AF_INET, &(client_addr.sin_addr), client_ip_str, INET_ADDRSTRLEN) == NULL) {
            strcpy(client_ip_str, "UNKNOWN");
        }

        cJSON *root = cJSON_Parse(buffer);
        if (!root) continue;

        cJSON *jid = cJSON_GetObjectItemCaseSensitive(root, "id");
        cJSON *jtemp = cJSON_GetObjectItemCaseSensitive(root, "temperature");
        cJSON *jlight = cJSON_GetObjectItemCaseSensitive(root, "light");
        cJSON *jseq = cJSON_GetObjectItemCaseSensitive(root, "seq");
        cJSON *jqos = cJSON_GetObjectItemCaseSensitive(root, "qos");

        if (!cJSON_IsString(jid) || !cJSON_IsNumber(jtemp) || !cJSON_IsNumber(jlight)) {
            cJSON_Delete(root);
            continue;
        }

        const char *id = jid->valuestring;
        double temp = jtemp->valuedouble;
        double light = jlight->valuedouble;
        long seq = (cJSON_IsNumber(jseq)) ? (long)jseq->valuedouble : -1;
        int qos = (cJSON_IsNumber(jqos)) ? jqos->valueint : 0;

        device_t *dev = add_or_get_device(id, &client_addr);
        
        if (qos == 1 && dev) {
            if (dev->has_seq && seq == dev->last_seq) {
                send_ack(sockfd, &client_addr, len, id, seq);
                cJSON_Delete(root);
                continue;
            }
        }

        if (dev) {
            dev->temperature = temp;
            dev->light = light;
            dev->last_seen = time(NULL);
            if (qos == 1) {
                dev->has_seq = 1;
                dev->last_seq = seq;
                send_ack(sockfd, &client_addr, len, id, seq);
            }
            printf("[%s] %s -> T:%.2f L:%.2f\n", client_ip_str, id, temp, light);

            if (temp < TEMP_MIN_VALID || temp > TEMP_MAX_VALID) {
                log_alert_dual(id, "TEMP_OOB", "Temperature reading nonsensical");
            }
        }

        assess_environment();
        cJSON_Delete(root);
    }
    return 0;
}