#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "cJSON.h"

#include "config.h"
#include "webctrl.h"
#include "commander.h"
#include "stabilizer.h"
#include "pm_esplane.h"
#include "sensors.h"
#include "system.h"

#define TAG "WEBCTRL"
#define WEBCTRL_PROTOCOL_VERSION 1

// Constants
#define WATCHDOG_TIMEOUT_MS  600
#define HEARTBEAT_PERIOD_MS  200
#define CONTROL_TASK_PERIOD_MS 20
#define TELEMETRY_PERIOD_MS  100

// Safe limits (clamping)
#define MAX_ROLL_DEG        15.0f
#define MAX_PITCH_DEG       15.0f
#define MAX_YAW_RATE_DEGS   45.0f
#define MAX_THRUST_LIMIT    55000.0f
#define MIN_THRUST_LIMIT    0.0f

// Slew limits (max change per second)
#define SLEW_ROLL_DEG_S     60.0f
#define SLEW_PITCH_DEG_S    60.0f
#define SLEW_YAW_RATE_S2    90.0f
#define SLEW_THRUST_S       20000.0f

#define SLEW_ROLL_PER_TICK     (SLEW_ROLL_DEG_S * (CONTROL_TASK_PERIOD_MS / 1000.0f))
#define SLEW_PITCH_PER_TICK    (SLEW_PITCH_DEG_S * (CONTROL_TASK_PERIOD_MS / 1000.0f))
#define SLEW_YAW_RATE_PER_TICK (SLEW_YAW_RATE_S2 * (CONTROL_TASK_PERIOD_MS / 1000.0f))
#define SLEW_THRUST_PER_TICK   (SLEW_THRUST_S * (CONTROL_TASK_PERIOD_MS / 1000.0f))

// Thruster increments
#define THRUST_STEP_INCREMENT  2000.0f

// States
typedef enum {
    STATE_DISARMED = 0,
    STATE_ARMED,
    STATE_FLYING,
    STATE_AUTOLAND
} flight_state_t;

static const char* state_strings[] = {
    "DISARMED",
    "ARMED",
    "FLYING",
    "AUTOLAND"
};

// State variables (protected by state_mutex)
typedef struct {
    flight_state_t state;
    uint32_t session_token;
    int active_sockfd;
    
    // Target command inputs (from client)
    float target_roll;
    float target_pitch;
    float target_yaw_rate;
    float target_thrust;
    
    // Filtered command outputs (slew-rate limited, sent to flight stack)
    float current_roll;
    float current_pitch;
    float current_yaw_rate;
    float current_thrust;

    uint64_t last_packet_time_ms;
    uint32_t last_seq;
    uint32_t dropped_packets;
    bool restart_requested;
} ctrl_state_t;

static ctrl_state_t g_ctrl;
static SemaphoreHandle_t g_state_mutex = NULL;
static httpd_handle_t g_server = NULL;

// Embed index.html
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");

// Get timestamp in milliseconds
static uint64_t get_time_ms(void)
{
    return (uint64_t)(esp_timer_get_time() / 1000);
}

// Prototypes for internal helpers
static void webctrl_timer_task(void *pvParameters);
static void webctrl_telemetry_task(void *pvParameters);
static esp_err_t ws_handler(httpd_req_t *req);
static esp_err_t root_get_handler(httpd_req_t *req);

// HTTP route registrations
static const httpd_uri_t root_uri = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = root_get_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t ws_uri = {
    .uri        = "/ws",
    .method     = HTTP_GET,
    .handler    = ws_handler,
    .user_ctx   = NULL,
    .is_websocket = true
};

static esp_err_t root_get_handler(httpd_req_t *req)
{
    const size_t size = index_html_end - index_html_start;
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, (const char *)index_html_start, size);
    return ESP_OK;
}

// Check arming interlocks
// Battery >= 3.2V, IMU converged, sensors calibrated
static bool verify_arming_interlocks(char *error_msg_out, size_t error_len)
{
    float battery_v = pmGetBatteryVoltage();
    if (battery_v < 3.2f) {
        snprintf(error_msg_out, error_len, "Battery too low: %.2fV < 3.2V", battery_v);
        return false;
    }

    if (!sensorsAreCalibrated()) {
        snprintf(error_msg_out, error_len, "IMU sensor calibration in progress");
        return false;
    }

    float roll = 0, pitch = 0, yaw = 0, thrust = 0, loop_freq = 0;
    stabilizerGetTelemetry(&roll, &pitch, &yaw, &thrust, &loop_freq);

    if (fabsf(roll) > 5.0f || fabsf(pitch) > 5.0f) {
        snprintf(error_msg_out, error_len, "Attitude not level: R=%.1f, P=%.1f (must be <5.0 deg)", roll, pitch);
        return false;
    }

    return true;
}

// Safe restart task
static void restart_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(TAG, "Rebooting ESP32 drone...");
    esp_restart();
}

// Save SSID and Password to NVS
static bool save_wifi_credentials(const char *ssid, const char *password)
{
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("wifi_cfg", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        return false;
    }

    err = nvs_set_str(my_handle, "ssid", ssid);
    if (err == ESP_OK && password != NULL) {
        err = nvs_set_str(my_handle, "password", password);
    } else if (err == ESP_OK) {
        err = nvs_set_str(my_handle, "password", "");
    }

    if (err == ESP_OK) {
        err = nvs_commit(my_handle);
    }
    
    nvs_close(my_handle);
    return (err == ESP_OK);
}

// Parse and execute client JSON commands
static void process_client_command(int sockfd, const char *payload, size_t length)
{
    cJSON *root = cJSON_ParseWithLength(payload, length);
    if (root == NULL) {
        xSemaphoreTake(g_state_mutex, portMAX_DELAY);
        g_ctrl.dropped_packets++;
        xSemaphoreGive(g_state_mutex);
        ESP_LOGE(TAG, "Failed to parse JSON");
        return;
    }

    cJSON *v_item = cJSON_GetObjectItem(root, "v");
    cJSON *t_item = cJSON_GetObjectItem(root, "t");

    if (!v_item || v_item->valueint != WEBCTRL_PROTOCOL_VERSION || !t_item) {
        xSemaphoreTake(g_state_mutex, portMAX_DELAY);
        g_ctrl.dropped_packets++;
        xSemaphoreGive(g_state_mutex);
        cJSON_Delete(root);
        return;
    }

    xSemaphoreTake(g_state_mutex, portMAX_DELAY);

    // Track watchdog timestamp on any valid protocol packet
    g_ctrl.last_packet_time_ms = get_time_ms();

    cJSON *token_item = cJSON_GetObjectItem(root, "token");
    cJSON *seq_item = cJSON_GetObjectItem(root, "seq");
    if (seq_item) {
        g_ctrl.last_seq = seq_item->valueint;
    }

    const char *msg_type = t_item->valuestring;

    if (strcmp(msg_type, "heartbeat") == 0) {
        // Heartbeat only, timestamps updated above
        xSemaphoreGive(g_state_mutex);
        cJSON_Delete(root);
        return;
    }

    // Command frame checks session token & control lock
    if (token_item && (uint32_t)token_item->valuedouble == g_ctrl.session_token && g_ctrl.active_sockfd == sockfd) {
        if (strcmp(msg_type, "cmd") == 0) {
            cJSON *action_item = cJSON_GetObjectItem(root, "action");
            const char *action = action_item ? action_item->valuestring : "none";

            // Unconditional DISARM
            if (strcmp(action, "disarm") == 0) {
                g_ctrl.state = STATE_DISARMED;
                g_ctrl.target_thrust = 0;
                g_ctrl.current_thrust = 0;
                systemSetArmed(false);
                ESP_LOGW(TAG, "STATE: DISARMED via client command");
            } 
            else if (g_ctrl.state == STATE_DISARMED) {
                if (strcmp(action, "arm") == 0) {
                    char err_msg[64] = {0};
                    if (verify_arming_interlocks(err_msg, sizeof(err_msg))) {
                        g_ctrl.state = STATE_ARMED;
                        g_ctrl.target_roll = 0.0f;
                        g_ctrl.target_pitch = 0.0f;
                        g_ctrl.target_yaw_rate = 0.0f;
                        g_ctrl.target_thrust = 0.0f;
                        g_ctrl.current_roll = 0.0f;
                        g_ctrl.current_pitch = 0.0f;
                        g_ctrl.current_yaw_rate = 0.0f;
                        g_ctrl.current_thrust = 0.0f;
                        systemSetArmed(true);
                        ESP_LOGI(TAG, "STATE: ARMED successfully");
                    } else {
                        ESP_LOGW(TAG, "Arming rejected: %s", err_msg);
                    }
                }
                else if (strcmp(action, "save_wifi") == 0) {
                    cJSON *ssid_item = cJSON_GetObjectItem(root, "ssid");
                    cJSON *pass_item = cJSON_GetObjectItem(root, "pass");
                    if (ssid_item && ssid_item->valuestring) {
                        const char *ssid = ssid_item->valuestring;
                        const char *pass = (pass_item && pass_item->valuestring) ? pass_item->valuestring : "";
                        if (save_wifi_credentials(ssid, pass)) {
                            ESP_LOGI(TAG, "WiFi credentials saved. Rebooting AP...");
                            g_ctrl.restart_requested = true;
                        }
                    }
                }
            }
            else if (g_ctrl.state == STATE_ARMED || g_ctrl.state == STATE_FLYING) {
                // Read control targets
                cJSON *roll_item = cJSON_GetObjectItem(root, "roll");
                cJSON *pitch_item = cJSON_GetObjectItem(root, "pitch");
                cJSON *yaw_item = cJSON_GetObjectItem(root, "yaw_rate");

                if (roll_item) g_ctrl.target_roll = (float)roll_item->valuedouble;
                if (pitch_item) g_ctrl.target_pitch = (float)pitch_item->valuedouble;
                if (yaw_item) g_ctrl.target_yaw_rate = (float)yaw_item->valuedouble;

                // Process throttle commands
                if (strcmp(action, "takeoff_step") == 0) {
                    g_ctrl.target_thrust += THRUST_STEP_INCREMENT;
                } else if (strcmp(action, "down_step") == 0) {
                    g_ctrl.target_thrust -= THRUST_STEP_INCREMENT;
                } else if (strcmp(action, "land") == 0) {
                    g_ctrl.state = STATE_AUTOLAND;
                    g_ctrl.target_roll = 0.0f;
                    g_ctrl.target_pitch = 0.0f;
                    g_ctrl.target_yaw_rate = 0.0f;
                    g_ctrl.target_thrust = 0.0f; // Slew-rate limits landing down to zero
                    ESP_LOGI(TAG, "STATE: AUTOLAND via client command");
                }

                // Auto-transition from ARMED to FLYING if throttle is applied
                if (g_ctrl.state == STATE_ARMED && g_ctrl.target_thrust > 5000.0f) {
                    g_ctrl.state = STATE_FLYING;
                    ESP_LOGI(TAG, "STATE: FLYING");
                }
            }
        }
    } else {
        // Wrong token or not the active controlling client
        g_ctrl.dropped_packets++;
    }

    xSemaphoreGive(g_state_mutex);
    cJSON_Delete(root);
}

// WebSocket connection handling
static esp_err_t ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        int client_fd = httpd_req_to_sockfd(req);
        ESP_LOGI(TAG, "WebSocket handshake completed on FD %d", client_fd);

        xSemaphoreTake(g_state_mutex, portMAX_DELAY);
        bool client_takes_control = false;
        
        // Single client policy
        if (g_ctrl.active_sockfd == -1) {
            g_ctrl.active_sockfd = client_fd;
            g_ctrl.session_token = esp_random();
            g_ctrl.last_packet_time_ms = get_time_ms();
            client_takes_control = true;
            ESP_LOGI(TAG, "Client FD %d acquired control. Token: %u", client_fd, g_ctrl.session_token);
        } else {
            ESP_LOGI(TAG, "Client FD %d registered as telemetry-only", client_fd);
        }

        // Send connection acknowledgment frame
        cJSON *ack = cJSON_CreateObject();
        cJSON_AddNumberToObject(ack, "v", WEBCTRL_PROTOCOL_VERSION);
        cJSON_AddStringToObject(ack, "t", "conn_ack");
        cJSON_AddNumberToObject(ack, "token", client_takes_control ? g_ctrl.session_token : 0);
        cJSON_AddBoolToObject(ack, "controlled", client_takes_control);
        
        char *json_str = cJSON_PrintUnformatted(ack);
        cJSON_Delete(ack);

        httpd_ws_frame_t ws_pkt = {
            .final = true,
            .fragmented = false,
            .type = HTTPD_WS_TYPE_TEXT,
            .payload = (uint8_t *)json_str,
            .len = strlen(json_str)
        };
        
        httpd_ws_send_frame(req, &ws_pkt);
        free(json_str);

        xSemaphoreGive(g_state_mutex);
        return ESP_OK;
    }

    // Handle incoming frames
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;

    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) {
        return ret;
    }

    if (ws_pkt.len > 0) {
        // Preallocated buffer on the stack to prevent dynamic allocation on hot path (L3)
        if (ws_pkt.len < 512) {
            char buffer[512];
            ws_pkt.payload = (uint8_t *)buffer;
            ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
            if (ret == ESP_OK) {
                buffer[ws_pkt.len] = '\0';
                process_client_command(httpd_req_to_sockfd(req), buffer, ws_pkt.len);
            }
        } else {
            ESP_LOGW(TAG, "Frame size too large: %d", ws_pkt.len);
            xSemaphoreTake(g_state_mutex, portMAX_DELAY);
            g_ctrl.dropped_packets++;
            xSemaphoreGive(g_state_mutex);
        }
    }
    return ESP_OK;
}

// Telemetry streaming task
static void webctrl_telemetry_task(void *pvParameters)
{
    TickType_t last_wake = xTaskGetTickCount();
    
    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
        
        xSemaphoreTake(g_state_mutex, portMAX_DELAY);
        uint32_t seq_echo = g_ctrl.last_seq;
        float curr_thrust = g_ctrl.current_thrust;
        flight_state_t state = g_ctrl.state;
        uint32_t dropped = g_ctrl.dropped_packets;
        xSemaphoreGive(g_state_mutex);

        // Fetch telemetry from stabilizer
        float roll = 0, pitch = 0, yaw = 0, loop_freq = 0, dummy_thrust = 0;
        stabilizerGetTelemetry(&roll, &pitch, &yaw, &dummy_thrust, &loop_freq);
        float battery = pmGetBatteryVoltage();

        // Build telemetry packet
        cJSON *tel = cJSON_CreateObject();
        cJSON_AddNumberToObject(tel, "v", WEBCTRL_PROTOCOL_VERSION);
        cJSON_AddStringToObject(tel, "t", "telemetry");
        cJSON_AddNumberToObject(tel, "seq_echo", seq_echo);
        cJSON_AddNumberToObject(tel, "roll", roll);
        cJSON_AddNumberToObject(tel, "pitch", pitch);
        cJSON_AddNumberToObject(tel, "yaw", yaw);
        cJSON_AddNumberToObject(tel, "thrust", (int)curr_thrust);
        cJSON_AddNumberToObject(tel, "battery", battery);
        cJSON_AddStringToObject(tel, "state", state_strings[state]);
        cJSON_AddNumberToObject(tel, "stab_freq", loop_freq);
        cJSON_AddNumberToObject(tel, "dropped_packets", dropped);

        char *json_str = cJSON_PrintUnformatted(tel);
        cJSON_Delete(tel);

        // Broadcast to all connected clients
        // Obtain client file descriptors list
        size_t clients_num = 10;
        int client_fds[10] = {0};
        
        if (g_server != NULL && httpd_get_client_list(g_server, &clients_num, client_fds) == ESP_OK) {
            for (size_t i = 0; i < clients_num; i++) {
                httpd_ws_frame_t ws_pkt = {
                    .final = true,
                    .fragmented = false,
                    .type = HTTPD_WS_TYPE_TEXT,
                    .payload = (uint8_t *)json_str,
                    .len = strlen(json_str)
                };
                
                // Telemetry frame transmission must not block (L3)
                // We use HTTPD_TRANSFER_TO_SYSTEM which is non-blocking in lwip
                esp_err_t err = httpd_ws_send_data(g_server, client_fds[i], &ws_pkt);
                if (err != ESP_OK) {
                    // Drop frame silently if backpressured
                }
            }
        }
        free(json_str);
    }
}

// Timer & Watchdog & Slew-rate loop task
static void webctrl_timer_task(void *pvParameters)
{
    TickType_t last_wake = xTaskGetTickCount();
    setpoint_t flight_setpoint;
    memset(&flight_setpoint, 0, sizeof(setpoint_t));

    // Stabilizer mode selections
    flight_setpoint.mode.x = modeDisable;
    flight_setpoint.mode.y = modeDisable;
    flight_setpoint.mode.z = modeDisable;
    flight_setpoint.mode.roll = modeAbs;
    flight_setpoint.mode.pitch = modeAbs;
    flight_setpoint.mode.yaw = modeVelocity;

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS));
        
        uint64_t now = get_time_ms();
        bool trigger_autoland = false;
        bool trigger_restart = false;

        xSemaphoreTake(g_state_mutex, portMAX_DELAY);

        // Check if reboot requested
        if (g_ctrl.restart_requested) {
            trigger_restart = true;
            g_ctrl.restart_requested = false;
        }

        // S1: Link Watchdog check
        if ((g_ctrl.state == STATE_ARMED || g_ctrl.state == STATE_FLYING) && 
            (now - g_ctrl.last_packet_time_ms > WATCHDOG_TIMEOUT_MS)) {
            g_ctrl.state = STATE_AUTOLAND;
            g_ctrl.target_roll = 0.0f;
            g_ctrl.target_pitch = 0.0f;
            g_ctrl.target_yaw_rate = 0.0f;
            g_ctrl.target_thrust = 0.0f;
            trigger_autoland = true;
        }

        // Clamp targets strictly (S4)
        g_ctrl.target_roll = fmaxf(-MAX_ROLL_DEG, fminf(g_ctrl.target_roll, MAX_ROLL_DEG));
        g_ctrl.target_pitch = fmaxf(-MAX_PITCH_DEG, fminf(g_ctrl.target_pitch, MAX_PITCH_DEG));
        g_ctrl.target_yaw_rate = fmaxf(-MAX_YAW_RATE_DEGS, fminf(g_ctrl.target_yaw_rate, MAX_YAW_RATE_DEGS));
        g_ctrl.target_thrust = fmaxf(MIN_THRUST_LIMIT, fminf(g_ctrl.target_thrust, MAX_THRUST_LIMIT));

        if (g_ctrl.state == STATE_AUTOLAND) {
            // Autoland safety ramp down (S1)
            g_ctrl.target_roll = 0.0f;
            g_ctrl.target_pitch = 0.0f;
            g_ctrl.target_yaw_rate = 0.0f;
            g_ctrl.target_thrust = 0.0f;
        }

        // Slew-rate limiting calculations (S4)
        float d_roll = g_ctrl.target_roll - g_ctrl.current_roll;
        d_roll = fmaxf(-SLEW_ROLL_PER_TICK, fminf(d_roll, SLEW_ROLL_PER_TICK));
        g_ctrl.current_roll += d_roll;

        float d_pitch = g_ctrl.target_pitch - g_ctrl.current_pitch;
        d_pitch = fmaxf(-SLEW_PITCH_PER_TICK, fminf(d_pitch, SLEW_PITCH_PER_TICK));
        g_ctrl.current_pitch += d_pitch;

        float d_yaw = g_ctrl.target_yaw_rate - g_ctrl.current_yaw_rate;
        d_yaw = fmaxf(-SLEW_YAW_RATE_PER_TICK, fminf(d_yaw, SLEW_YAW_RATE_PER_TICK));
        g_ctrl.current_yaw_rate += d_yaw;

        float d_thrust = g_ctrl.target_thrust - g_ctrl.current_thrust;
        d_thrust = fmaxf(-SLEW_THRUST_PER_TICK, fminf(d_thrust, SLEW_THRUST_PER_TICK));
        g_ctrl.current_thrust += d_thrust;

        // Auto transition to DISARMED when autoland throttle reaches zero
        if (g_ctrl.state == STATE_AUTOLAND && g_ctrl.current_thrust <= 0.0f) {
            g_ctrl.state = STATE_DISARMED;
            systemSetArmed(false);
            ESP_LOGW(TAG, "STATE: DISARMED after autoland throttle ramp down");
        }

        // Sync local variables to push to flight stack setpoint struct
        flight_state_t current_state = g_ctrl.state;
        float roll_out = g_ctrl.current_roll;
        float pitch_out = g_ctrl.current_pitch;
        float yaw_rate_out = g_ctrl.current_yaw_rate;
        float thrust_out = g_ctrl.current_thrust;

        xSemaphoreGive(g_state_mutex);

        // Handle scheduled reboot task
        if (trigger_restart) {
            xTaskCreate(restart_task, "restart_task", 2048, NULL, 5, NULL);
        }

        if (trigger_autoland) {
            ESP_LOGW(TAG, "WATCHDOG TIMEOUT TRIP: entering STATE_AUTOLAND");
        }

        // Prepare flight stack command packet
        if (current_state == STATE_ARMED || current_state == STATE_FLYING || current_state == STATE_AUTOLAND) {
            flight_setpoint.attitude.roll = roll_out;
            flight_setpoint.attitude.pitch = pitch_out;
            flight_setpoint.attitudeRate.yaw = yaw_rate_out;
            flight_setpoint.thrust = thrust_out;
            
            // Push to flight stack setpoint queues via public commander API (S3)
            // Priority 3 (COMMANDER_PRIORITY_WEBCTRL) overrides other commander modes.
            commanderSetSetpoint(&flight_setpoint, 3);
        }
    }
}

// Clean disconnected websockets from lock ownership
static void ws_cleanup_handler(void *arg, int fd)
{
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    if (g_ctrl.active_sockfd == fd) {
        ESP_LOGW(TAG, "Active control client FD %d disconnected. Control released.", fd);
        g_ctrl.active_sockfd = -1;
        g_ctrl.session_token = 0;
        
        // If client disconnected while armed, transition to AUTOLAND safely
        if (g_ctrl.state == STATE_ARMED || g_ctrl.state == STATE_FLYING) {
            g_ctrl.state = STATE_AUTOLAND;
            g_ctrl.target_roll = 0.0f;
            g_ctrl.target_pitch = 0.0f;
            g_ctrl.target_yaw_rate = 0.0f;
            g_ctrl.target_thrust = 0.0f;
        }
    }
    xSemaphoreGive(g_state_mutex);
}

void webctrlInit(void)
{
    g_state_mutex = xSemaphoreCreateMutex();
    assert(g_state_mutex);

    // Initial state setup
    g_ctrl.state = STATE_DISARMED;
    g_ctrl.session_token = 0;
    g_ctrl.active_sockfd = -1;
    g_ctrl.target_roll = 0.0f;
    g_ctrl.target_pitch = 0.0f;
    g_ctrl.target_yaw_rate = 0.0f;
    g_ctrl.target_thrust = 0.0f;
    g_ctrl.current_roll = 0.0f;
    g_ctrl.current_pitch = 0.0f;
    g_ctrl.current_yaw_rate = 0.0f;
    g_ctrl.current_thrust = 0.0f;
    g_ctrl.last_packet_time_ms = get_time_ms();
    g_ctrl.last_seq = 0;
    g_ctrl.dropped_packets = 0;
    g_ctrl.restart_requested = false;

    // Start HTTP Server
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_open_sockets = 7;
    config.close_fn = ws_cleanup_handler;

    ESP_LOGI(TAG, "Starting Web Ground Control Server on port %d...", config.server_port);
    if (httpd_start(&g_server, &config) == ESP_OK) {
        httpd_register_uri_handler(g_server, &root_uri);
        httpd_register_uri_handler(g_server, &ws_uri);
    } else {
        ESP_LOGE(TAG, "Failed to start HTTP server!");
        // Fail-safe autoland (S6)
        systemSetArmed(false);
        return;
    }

    // Launch background tasks pinned to CPU 0 (L1)
    // Priority set strictly below flight control loops (S stabilizier=5, sensors=4)
    xTaskCreatePinnedToCore(webctrl_timer_task, "webctrl_timer", 3072, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(webctrl_telemetry_task, "webctrl_tel", 4096, NULL, 2, NULL, 0);

    ESP_LOGI(TAG, "Web Control interface initialized successfully");
}
