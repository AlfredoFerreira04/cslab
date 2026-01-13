// alert_udp_server.c
// Compile: gcc alert_udp_server.c asm.s -o alert_udp_server -lcjson -lpaho-mqtt3c -lpthread
// Run: ./alert_udp_server

#include "asm.h" // Must correspond to asm.s containing check_limit
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
#include <signal.h>

// --- CONFIGURATION ---
#define PORT 5005
#define BUFFER_SIZE 8192
#define MAX_DEVICES 1024
#define ALERT_LOGFILE "alerts.log"

// Validation
#define TEMP_MIN_VALID -50.0
#define TEMP_MAX_VALID 100.0

// PREFERENCES & HYSTERESIS
#define DEFAULT_PREF_TEMP_MIN 20.0
#define DEFAULT_PREF_TEMP_MAX 26.0
#define DEFAULT_PREF_LIGHT_MIN 5.0
#define OVERLOARD_LIGHT_THRESHOLD 3.0

// Alert Thresholds
#define TEMP_DIFF_THRESHOLD 3.0

// --- TIMING CONFIGURATION ---
#define COMMAND_COOLDOWN 0.5        // Global Hardware cooldown (0.5s)
#define ACTUATOR_SEQUENCE_TIME 45.0 // LOCKOUT: Retry interval (45s)

#define INACTIVITY_TIMEOUT_SEC 15
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

// --- STATE TRACKING ---
int last_sent_heat_state = -1;
int last_sent_light_state = -1;

time_t last_global_command_time = 0;

// --- INDEPENDENT LOCKOUT TIMERS ---
time_t heat_lockout_until = 0;
time_t light_lockout_until = 0;

volatile sig_atomic_t stop_server = 0;

// Prototypes
static void log_alert(const char *message);
static device_t *add_or_get_device(const char *id, struct sockaddr_in *addr);
static void send_ack(int sockfd, struct sockaddr_in *client_addr, socklen_t addrlen, const char *id, long seq);
void send_actuator_command(const char *type, const char *action);

void handle_sigint(int sig) { stop_server = 1; }

static void log_alert(const char *message)
{
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

void *mqtt_thread_func(void *arg)
{
    while (!stop_server)
    {
        MQTTClient_yield();
        usleep(100 * 1000);
    }
    return NULL;
}

void select_actuator()
{
    if (device_count > 0 && strlen(designated_actuator_id) == 0)
    {
        strncpy(designated_actuator_id, devices[0].id, sizeof(designated_actuator_id) - 1);
        char msg[256];
        snprintf(msg, sizeof(msg), "SYSTEM: Designated %s as the primary ACTUATOR.", designated_actuator_id);
        log_alert(msg);
        last_sent_heat_state = -1;
        last_sent_light_state = -1;
    }
}

// --- UPDATED COMMAND LOGIC (CONTINUOUS RETRY) ---
void send_actuator_command(const char *type, const char *action)
{
    if (strlen(designated_actuator_id) == 0)
        return;

    time_t now = time(NULL);

    // 1. Hardware Cooldown Check (0.5s) - Protects Radio
    if (!stop_server && difftime(now, last_global_command_time) < COMMAND_COOLDOWN)
        return;

    // 2. Construct MQTT Packet
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

    // 3. Log and Reset Lockout
    char logbuf[256];
    int desired_state = (strcmp(action, "ON") == 0) ? 1 : 0;

    const char *status_tag = "ORDER SENT";
    if ((strcmp(type, "HEAT") == 0 && last_sent_heat_state == desired_state) ||
        (strcmp(type, "LIGHT") == 0 && last_sent_light_state == desired_state))
    {
        status_tag = "RETRYING";
    }

    snprintf(logbuf, sizeof(logbuf), "%s: %s -> %s %s", status_tag, designated_actuator_id, type, action);
    log_alert(logbuf);

    if (strcmp(type, "HEAT") == 0)
    {
        heat_lockout_until = now + (time_t)ACTUATOR_SEQUENCE_TIME;
        last_sent_heat_state = desired_state;
        printf("SYSTEM: Heat locked for %.0fs (Waiting for check...)\n", ACTUATOR_SEQUENCE_TIME);
    }
    else if (strcmp(type, "LIGHT") == 0)
    {
        light_lockout_until = now + (time_t)ACTUATOR_SEQUENCE_TIME;
        last_sent_light_state = desired_state;
        printf("SYSTEM: Light locked for %.0fs (Waiting for check...)\n", ACTUATOR_SEQUENCE_TIME);
    }

    // 4. Update Global Timer
    last_global_command_time = now;

    free(json_str);
    cJSON_Delete(root);
}

// --- UPDATED SENSOR LOGIC ---
void assess_environment()
{
    if (device_count == 0)
        return;

    time_t now = time(NULL);

    double sum_temp = 0;
    double sum_light = 0;
    int valid_count = 0;

    for (int i = 0; i < device_count; i++)
    {
        if (devices[i].temperature > TEMP_MIN_VALID && devices[i].temperature < TEMP_MAX_VALID)
        {
            sum_temp += devices[i].temperature;
            sum_light += devices[i].light;
            valid_count++;
        }
    }

    if (valid_count == 0)
        return;

    double avg_temp = sum_temp / valid_count;
    double avg_light = sum_light / valid_count;

    // --- INDEPENDENT DECISION LOGIC ---

    // 1. HEAT LOGIC
    if (now >= heat_lockout_until)
    {
        if (avg_temp < pref_temp_min)
            send_actuator_command("HEAT", "ON");
        else if (avg_temp > pref_temp_max)
            send_actuator_command("HEAT", "OFF");
    }

    // 2. LIGHT LOGIC
    if (now >= light_lockout_until)
    {
        if(pref_light_min + OVERLOARD_LIGHT_THRESHOLD < avg_light){
            send_actuator_command("LIGHT", "OFF");
        }else if (avg_light < pref_light_min){
            send_actuator_command("LIGHT", "ON");
        }
    }
}

static void log_alert_dual(const char *device, const char *alert_type, const char *message)
{
    char formatted[512];
    snprintf(formatted, sizeof(formatted), "%s: device=%s: %s", alert_type, device, message);
    log_alert(formatted);
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
    free(json_str);
}

void *monitor_device_status(void *arg)
{
    time_t current_time;
    char message[256];
    while (!stop_server)
    {
        sleep(MONITOR_INTERVAL_SEC);
        current_time = time(NULL);
        for (int i = 0; i < device_count; ++i)
        {
            double duration = difftime(current_time, devices[i].last_seen);
            if (duration > INACTIVITY_TIMEOUT_SEC)
            {
                snprintf(message, sizeof(message), "Client silent for %.0f sec.", duration);
                log_alert_dual(devices[i].id, "CLIENT_INACTIVITY", message);

                if (strcmp(devices[i].id, designated_actuator_id) == 0)
                {
                    log_alert("SYSTEM: Actuator is unresponsive. Resetting designation.");
                    designated_actuator_id[0] = '\0';
                    last_sent_heat_state = -1; // Reset so next actuator gets commands
                    last_sent_light_state = -1;
                }
            }
        }
    }
    return NULL;
}

static device_t *find_device_by_id(const char *id)
{
    for (int i = 0; i < device_count; ++i)
        if (strcmp(devices[i].id, id) == 0)
            return &devices[i];
    return NULL;
}

static device_t *add_or_get_device(const char *id, struct sockaddr_in *addr)
{
    device_t *d = find_device_by_id(id);
    if (d)
    {
        d->addr = *addr;
        d->last_seen = time(NULL);
        return d;
    }
    if (device_count >= MAX_DEVICES)
        return NULL;
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
    if (!id)
        return;
    cJSON *ack = cJSON_CreateObject();
    cJSON_AddStringToObject(ack, "type", "ACK");
    cJSON_AddStringToObject(ack, "id", id);
    cJSON_AddNumberToObject(ack, "seq", (double)seq);
    char *out = cJSON_PrintUnformatted(ack);
    if (out)
    {
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

    if (strcmp(topicName, MQTT_PREF_TEMP_TOPIC) == 0)
    {
        double value = atoi(payload);
        pref_temp_min = value - TEMP_DIFF_THRESHOLD;
        pref_temp_max = value + TEMP_DIFF_THRESHOLD;
        printf("PREFS: Updated Temp Range: %.1f - %.1f\n", pref_temp_min, pref_temp_max);
    }
    else if (strcmp(topicName, MQTT_PREF_LIGHT_TOPIC) == 0)
    {
        double min;
        if (sscanf(payload, "%lf", &min) == 1)
        {
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

    signal(SIGINT, handle_sigint);

    alert_log = fopen(ALERT_LOGFILE, "a");
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
        exit(EXIT_FAILURE);
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(PORT);

    if (bind(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    {
        perror("Bind failed");
        exit(EXIT_FAILURE);
    }

    printf("Server running on port %d. Press Ctrl+C to Shutdown.\n", PORT);

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

    if (MQTTClient_connect(client, &conn_opts) == MQTTCLIENT_SUCCESS)
    {
        printf("Connected to MQTT.\n");
        MQTTClient_subscribe(client, MQTT_PREF_LIGHT_TOPIC, 1);
        MQTTClient_subscribe(client, MQTT_PREF_TEMP_TOPIC, 1);
        pthread_create(&mqtt_thread, NULL, mqtt_thread_func, NULL);
    }

    pthread_create(&monitor_thread, NULL, monitor_device_status, NULL);

    while (!stop_server)
    {
        socklen_t len = sizeof(client_addr);
        ssize_t n = recvfrom(sockfd, buffer, BUFFER_SIZE - 1, 0, (struct sockaddr *)&client_addr, &len);

        if (n < 0)
        {
            if (errno == EINTR && stop_server)
                break;
            continue;
        }

        buffer[n] = '\0';
        inet_ntop(AF_INET, &(client_addr.sin_addr), client_ip_str, INET_ADDRSTRLEN);

        cJSON *root = cJSON_Parse(buffer);
        if (!root)
            continue;

        cJSON *jid = cJSON_GetObjectItemCaseSensitive(root, "id");
        cJSON *jtemp = cJSON_GetObjectItemCaseSensitive(root, "temperature");
        cJSON *jlight = cJSON_GetObjectItemCaseSensitive(root, "light");
        cJSON *jseq = cJSON_GetObjectItemCaseSensitive(root, "seq");
        cJSON *jqos = cJSON_GetObjectItemCaseSensitive(root, "qos");

        if (cJSON_IsString(jid) && cJSON_IsNumber(jtemp))
        {
            const char *id = jid->valuestring;
            double temp = jtemp->valuedouble;
            double light = cJSON_IsNumber(jlight) ? jlight->valuedouble : 0;
            long seq = (cJSON_IsNumber(jseq)) ? (long)jseq->valuedouble : -1;
            int qos = (cJSON_IsNumber(jqos)) ? jqos->valueint : 0;

            device_t *dev = add_or_get_device(id, &client_addr);

            int duplicate = (qos == 1 && dev && dev->has_seq && seq == dev->last_seq);
            if (duplicate)
            {
                send_ack(sockfd, &client_addr, len, id, seq);
            }
            else if (dev)
            {
                dev->temperature = temp;
                dev->light = light;
                dev->last_seen = time(NULL);
                if (qos == 1)
                {
                    dev->has_seq = 1;
                    dev->last_seq = seq;
                    send_ack(sockfd, &client_addr, len, id, seq);
                }
                printf("[%s] -> T:%.2f L:%.2f\n", id, temp, light);

                if (temp < TEMP_MIN_VALID || temp > TEMP_MAX_VALID)
                {
                    log_alert_dual(id, "TEMP_OOB", "Temperature reading nonsensical");
                }

                // --- CHECK_LIMIT (ASM) INTRODUCED HERE ---
                for (int i = 0; i < device_count; ++i)
                {
                    device_t *other = &devices[i];
                    if (strcmp(other->id, dev->id) == 0)
                        continue;

                    // Call the Assembly function to check diff > threshold
                    int result = check_limit(dev->temperature, other->temperature, TEMP_DIFF_THRESHOLD);

                    if (result == 1)
                    {
                        char msg[256];
                        snprintf(msg, sizeof(msg),
                                 "Differential Alert: %.2f vs %.2f (Threshold: %.2f) compared to %s",
                                 dev->temperature, other->temperature, TEMP_DIFF_THRESHOLD, other->id);
                        log_alert_dual(id, "DIFFERENTIAL_ALERT", msg);
                    }
                }
            }
        }
        assess_environment();
        cJSON_Delete(root);
    }

    printf("\n*** SHUTTING DOWN ***\nSending EMERGENCY OFF...\n");
    last_sent_heat_state = -1;
    last_sent_light_state = -1;
    send_actuator_command("HEAT", "OFF");
    send_actuator_command("LIGHT", "OFF");

    printf("Waiting for MQTT delivery...\n");
    sleep(2);

    if (alert_log)
        fclose(alert_log);
    MQTTClient_disconnect(client, 1000);
    MQTTClient_destroy(&client);
    close(sockfd);
    printf("Goodbye.\n");
    return 0;
}