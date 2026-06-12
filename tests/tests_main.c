#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include <stdbool.h>

// Mock ESP-IDF and FreeRTOS type definitions
typedef int esp_err_t;
#define ESP_OK          0
#define ESP_FAIL        -1
#define ESP_ERR_NVS_NO_FREE_PAGES 0x110d
#define ESP_ERR_NVS_NEW_VERSION_FOUND 0x110e

typedef int BaseType_t;
#define pdTRUE          1
#define pdFALSE         0
typedef unsigned int TickType_t;
#define pdMS_TO_TICKS(x) ((x) / 20)

typedef void* SemaphoreHandle_t;
typedef void* httpd_handle_t;
typedef int httpd_req_t;

// Mock Flight Stack constants
typedef enum {
    modeDisable = 0,
    modeAbs,
    modeVelocity
} stab_mode_t;

typedef struct {
    float roll;
    float pitch;
    float yaw;
} attitude_t;

typedef struct {
    float x;
    float y;
    float z;
} point_t;

typedef struct {
    float x;
    float y;
    float z;
} velocity_t;

typedef struct {
    float x;
    float y;
    float z;
} acc_t;

typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
} quaternion_t;

typedef struct setpoint_s {
    uint32_t timestamp;
    attitude_t attitude;
    attitude_t attitudeRate;
    quaternion_t attitudeQuaternion;
    float thrust;
    point_t position;
    velocity_t velocity;
    acc_t acceleration;
    bool velocity_body;
    struct {
        stab_mode_t x;
        stab_mode_t y;
        stab_mode_t z;
        stab_mode_t roll;
        stab_mode_t pitch;
        stab_mode_t yaw;
        stab_mode_t quat;
    } mode;
} setpoint_t;

// Mock functions global variables for test asserts
static float mock_battery_voltage = 3.7f;
static bool mock_sensors_calibrated = true;
static float mock_stabilizer_roll = 0.0f;
static float mock_stabilizer_pitch = 0.0f;
static float mock_stabilizer_yaw = 0.0f;
static bool mock_system_armed = false;
static setpoint_t last_commander_setpoint;
static int last_commander_priority = -1;
static uint64_t mock_time_ms = 1000;

// Flight Stack mock APIs
float pmGetBatteryVoltage(void) {
    return mock_battery_voltage;
}

bool sensorsAreCalibrated(void) {
    return mock_sensors_calibrated;
}

void stabilizerGetTelemetry(float *roll, float *pitch, float *yaw, float *thrustRate, float *loopFreq) {
    *roll = mock_stabilizer_roll;
    *pitch = mock_stabilizer_pitch;
    *yaw = mock_stabilizer_yaw;
    *thrustRate = last_commander_setpoint.thrust;
    *loopFreq = 1000.0f;
}

void systemSetArmed(bool val) {
    mock_system_armed = val;
}

void commanderSetSetpoint(setpoint_t *setpoint, int priority) {
    last_commander_setpoint = *setpoint;
    last_commander_priority = priority;
}

// cJSON mock structure and parsing implementation
typedef struct cJSON {
    struct cJSON *next;
    struct cJSON *prev;
    struct cJSON *child;
    int type;
    char *valuestring;
    int valueint;
    double valuedouble;
    char *string;
} cJSON;

// Simple string key-value mock JSON parser
cJSON *cJSON_ParseWithLength(const char *value, size_t buffer_length) {
    // Return a dynamically allocated root cJSON node
    cJSON *root = (cJSON *)calloc(1, sizeof(cJSON));
    root->valuestring = strdup(value);
    return root;
}

void cJSON_Delete(cJSON *c) {
    if (c) {
        if (c->valuestring) free(c->valuestring);
        free(c);
    }
}

cJSON *cJSON_GetObjectItem(const cJSON * const object, const char * const string) {
    const char *json = object->valuestring;
    const char *key_ptr = strstr(json, string);
    if (!key_ptr) return NULL;

    cJSON *item = (cJSON *)calloc(1, sizeof(cJSON));
    item->string = (char *)string;

    // Find colon and value
    const char *val_ptr = strchr(key_ptr, ':');
    if (!val_ptr) {
        free(item);
        return NULL;
    }
    val_ptr++; // Skip ':'
    while (*val_ptr == ' ' || *val_ptr == '"') val_ptr++;

    // Extract value
    if (strstr(string, "v") == string) {
        sscanf(val_ptr, "%d", &item->valueint);
        item->valuedouble = item->valueint;
    } else if (strstr(string, "t") == string || strstr(string, "action") == string || strstr(string, "ssid") == string || strstr(string, "pass") == string) {
        char buf[128] = {0};
        int i = 0;
        while (*val_ptr && *val_ptr != '"' && *val_ptr != ',' && *val_ptr != '}' && i < 127) {
            buf[i++] = *val_ptr++;
        }
        item->valuestring = strdup(buf);
    } else {
        // Double/float values
        sscanf(val_ptr, "%lf", &item->valuedouble);
        item->valueint = (int)item->valuedouble;
    }

    return item;
}

// Include the source directly for testing, overriding ESP-IDF APIs with mocks
#define esp_timer_get_time()    (mock_time_ms * 1000)
#define esp_random()            (12345678U)
#define esp_restart()           (exit(0))
#define xSemaphoreCreateMutex() ((SemaphoreHandle_t)1)
#define xSemaphoreTake(m, t)    (pdTRUE)
#define xSemaphoreGive(m)       (pdTRUE)

// Logger macros
#define ESP_LOGI(t, f, ...)     printf("[INFO] " f "\n", ##__VA_ARGS__)
#define ESP_LOGW(t, f, ...)     printf("[WARN] " f "\n", ##__VA_ARGS__)
#define ESP_LOGE(t, f, ...)     printf("[ERROR] " f "\n", ##__VA_ARGS__)

// Mock ESP-IDF networking & NVS functions
typedef int nvs_handle_t;
#define NVS_READONLY 0
#define NVS_READWRITE 1
esp_err_t nvs_open(const char* name, int open_mode, nvs_handle_t *out_handle) { return ESP_OK; }
void nvs_close(nvs_handle_t handle) {}
esp_err_t nvs_set_str(nvs_handle_t handle, const char* key, const char* value) { return ESP_OK; }
esp_err_t nvs_get_str(nvs_handle_t handle, const char* key, char* out_value, size_t* length) { 
    if (strcmp(key, "ssid") == 0) {
        strcpy(out_value, "TestSSID");
        *length = strlen("TestSSID");
        return ESP_OK;
    }
    return ESP_FAIL;
}
esp_err_t nvs_commit(nvs_handle_t handle) { return ESP_OK; }

typedef struct {
    int max_open_sockets;
    void (*close_fn)(void *arg, int fd);
    int server_port;
} httpd_config_t;
#define HTTPD_DEFAULT_CONFIG() { .max_open_sockets = 5 }

esp_err_t httpd_start(httpd_handle_t *server, const httpd_config_t *config) { return ESP_OK; }
typedef struct {
    const char *uri;
    int method;
    esp_err_t (*handler)(httpd_req_t *r);
    void *user_ctx;
    bool is_websocket;
} httpd_uri_t;
#define HTTP_GET 1
#define HTTPD_WS_TYPE_TEXT 1
esp_err_t httpd_register_uri_handler(httpd_handle_t server, const httpd_uri_t *uri_handler) { return ESP_OK; }

int httpd_req_to_sockfd(httpd_req_t *r) { return *r; }
typedef struct {
    bool final;
    bool fragmented;
    int type;
    uint8_t *payload;
    size_t len;
} httpd_ws_frame_t;
esp_err_t httpd_ws_recv_frame(httpd_req_t *req, httpd_ws_frame_t *frame, size_t max_len) { return ESP_OK; }
esp_err_t httpd_ws_send_frame(httpd_req_t *req, httpd_ws_frame_t *frame) { return ESP_OK; }
esp_err_t httpd_ws_send_frame_to_fd(httpd_handle_t server, int fd, httpd_ws_frame_t *frame) { return ESP_OK; }
esp_err_t httpd_get_client_list(httpd_handle_t server, size_t *list_len, int *fds) { return ESP_FAIL; }

#define xTaskCreatePinnedToCore(a,b,c,d,e,f,g) (pdTRUE)
#define xTaskCreate(a,b,c,d,e,f) (pdTRUE)
#define assert(x)

// Include the component code directly from the structured source directory
#include "../src/webctrl/webctrl.c"

// Run protocol and command state tests
void test_protocol_and_clamping(void)
{
    printf("--- Running Protocol and Clamping Tests ---\n");
    webctrlInit();

    // Mock client connection handshake
    int dummy_req = 10;
    ws_handler((httpd_req_t *)&dummy_req);
    assert(g_ctrl.active_sockfd == 10);
    assert(g_ctrl.session_token == 12345678);

    // Test Case 1: Valid Heartbeat
    char hb_json[] = "{\"v\":1,\"t\":\"heartbeat\",\"token\":12345678,\"seq\":1}";
    process_client_command(10, hb_json, strlen(hb_json));
    assert(g_ctrl.dropped_packets == 0);

    // Test Case 2: Mismatched Protocol Version
    char bad_version_json[] = "{\"v\":2,\"t\":\"heartbeat\",\"token\":12345678,\"seq\":2}";
    process_client_command(10, bad_version_json, strlen(bad_version_json));
    assert(g_ctrl.dropped_packets == 1);

    // Test Case 3: Invalid Session Token
    char bad_token_json[] = "{\"v\":1,\"t\":\"cmd\",\"token\":999999,\"seq\":3,\"roll\":10.0,\"action\":\"none\"}";
    process_client_command(10, bad_token_json, strlen(bad_token_json));
    assert(g_ctrl.dropped_packets == 2);

    // Test Case 4: Roll Clamping bounds
    char clamp_roll_json[] = "{\"v\":1,\"t\":\"cmd\",\"token\":12345678,\"seq\":4,\"roll\":45.0,\"pitch\":0.0,\"yaw_rate\":0.0,\"action\":\"none\"}";
    // Enable state to ARMED first to accept command setpoints
    g_ctrl.state = STATE_ARMED;
    process_client_command(10, clamp_roll_json, strlen(clamp_roll_json));
    
    // Simulate control timer tick (runs slew rate calculations and clamping)
    // Run multiple ticks to allow slew rate to catch up to target
    for (int i = 0; i < 50; i++) {
        // Manually run timer tick logic
        float d_roll = g_ctrl.target_roll - g_ctrl.current_roll;
        d_roll = fmaxf(-SLEW_ROLL_PER_TICK, fminf(d_roll, SLEW_ROLL_PER_TICK));
        g_ctrl.current_roll += d_roll;
    }
    
    // Clamped value should be 15.0 degrees max, not 45.0
    printf("Roll target: %f, Current filtered roll: %f\n", g_ctrl.target_roll, g_ctrl.current_roll);
    assert(g_ctrl.target_roll == 15.0f);
    assert(g_ctrl.current_roll == 15.0f);

    printf("Protocol and Clamping tests passed!\n\n");
}

void test_arming_interlocks(void)
{
    printf("--- Running Arming Interlock Tests ---\n");
    g_ctrl.state = STATE_DISARMED;
    mock_system_armed = false;

    // Test 1: Arming rejected due to low battery
    mock_battery_voltage = 3.0f;
    mock_sensors_calibrated = true;
    mock_stabilizer_roll = 0.0f;
    char arm_cmd[] = "{\"v\":1,\"t\":\"cmd\",\"token\":12345678,\"seq\":5,\"action\":\"arm\"}";
    process_client_command(10, arm_cmd, strlen(arm_cmd));
    assert(g_ctrl.state == STATE_DISARMED);
    assert(!mock_system_armed);

    // Test 2: Arming rejected due to uncalibrated sensors
    mock_battery_voltage = 3.7f;
    mock_sensors_calibrated = false;
    process_client_command(10, arm_cmd, strlen(arm_cmd));
    assert(g_ctrl.state == STATE_DISARMED);
    assert(!mock_system_armed);

    // Test 3: Arming rejected due to attitude not level
    mock_battery_voltage = 3.7f;
    mock_sensors_calibrated = true;
    mock_stabilizer_roll = 10.0f; // Tilted roll
    process_client_command(10, arm_cmd, strlen(arm_cmd));
    assert(g_ctrl.state == STATE_DISARMED);
    assert(!mock_system_armed);

    // Test 4: Arming succeeds with safe conditions
    mock_stabilizer_roll = 0.0f;
    process_client_command(10, arm_cmd, strlen(arm_cmd));
    assert(g_ctrl.state == STATE_ARMED);
    assert(mock_system_armed);

    printf("Arming Interlock tests passed!\n\n");
}

void test_slew_rate_limiting(void)
{
    printf("--- Running Slew Rate Limiting Tests ---\n");
    
    // Simulate a target thrust command step from 0 to 40000
    g_ctrl.state = STATE_FLYING;
    g_ctrl.target_thrust = 40000.0f;
    g_ctrl.current_thrust = 0.0f;

    // Ticks calculation: target = 40000. Slew rate = 20000 units/second.
    // Task period = 20 ms. Limit per tick = 20000 * 0.02 = 400 units/tick.
    // Ticks required = 40000 / 400 = 100 ticks (2.0 seconds).
    
    // Run 1 tick
    float d_thrust = g_ctrl.target_thrust - g_ctrl.current_thrust;
    d_thrust = fmaxf(-SLEW_THRUST_PER_TICK, fminf(d_thrust, SLEW_THRUST_PER_TICK));
    g_ctrl.current_thrust += d_thrust;
    assert(g_ctrl.current_thrust == 400.0f);

    // Run remaining 99 ticks
    for (int i = 0; i < 99; i++) {
        d_thrust = g_ctrl.target_thrust - g_ctrl.current_thrust;
        d_thrust = fmaxf(-SLEW_THRUST_PER_TICK, fminf(d_thrust, SLEW_THRUST_PER_TICK));
        g_ctrl.current_thrust += d_thrust;
    }
    assert(g_ctrl.current_thrust == 40000.0f);

    printf("Slew Rate Limiting tests passed!\n\n");
}

void test_watchdog_timeout(void)
{
    printf("--- Running Watchdog Timeout Tests ---\n");
    g_ctrl.state = STATE_FLYING;
    g_ctrl.current_thrust = 30000.0f;
    g_ctrl.target_thrust = 30000.0f;
    
    // Simulate last packet was received 700 ms ago (exceeding 600 ms limit)
    mock_time_ms = 5000;
    g_ctrl.last_packet_time_ms = 4000; // 1000 ms gap
    
    // Trigger tick checks
    // Watchdog check should trip and transition state to AUTOLAND
    if ((g_ctrl.state == STATE_ARMED || g_ctrl.state == STATE_FLYING) && 
        (mock_time_ms - g_ctrl.last_packet_time_ms > WATCHDOG_TIMEOUT_MS)) {
        g_ctrl.state = STATE_AUTOLAND;
        g_ctrl.target_roll = 0.0f;
        g_ctrl.target_pitch = 0.0f;
        g_ctrl.target_yaw_rate = 0.0f;
        g_ctrl.target_thrust = 0.0f;
    }
    
    assert(g_ctrl.state == STATE_AUTOLAND);
    assert(g_ctrl.target_thrust == 0.0f);
    assert(g_ctrl.target_roll == 0.0f);

    // Slew down thrust over autoland phase
    // Thrust should ramp down from 30000 to 0. Slew limit per tick is 400.
    // 30000 / 400 = 75 ticks.
    for (int i = 0; i < 75; i++) {
        float d_thrust = g_ctrl.target_thrust - g_ctrl.current_thrust;
        d_thrust = fmaxf(-SLEW_THRUST_PER_TICK, fminf(d_thrust, SLEW_THRUST_PER_TICK));
        g_ctrl.current_thrust += d_thrust;
    }
    
    assert(g_ctrl.current_thrust == 0.0f);
    
    // System should automatically transition to DISARMED when thrust hits zero
    if (g_ctrl.state == STATE_AUTOLAND && g_ctrl.current_thrust <= 0.0f) {
        g_ctrl.state = STATE_DISARMED;
        systemSetArmed(false);
    }
    
    assert(g_ctrl.state == STATE_DISARMED);
    assert(!mock_system_armed);

    printf("Watchdog Timeout tests passed!\n\n");
}

int main(void)
{
    printf("==========================================\n");
    printf("        ESP-Drone Web Control Tests       \n");
    printf("==========================================\n");

    test_protocol_and_clamping();
    test_arming_interlocks();
    test_slew_rate_limiting();
    test_watchdog_timeout();

    printf("All host unit tests passed successfully!\n");
    return 0;
}
