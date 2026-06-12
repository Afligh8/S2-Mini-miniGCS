# ESP32 GCS (Ground Control Station)

[![Platform](https://img.shields.io/badge/Platform-ESP32--S2-orange.svg)](https://docs.espressif.com/projects/esp-idf/en/release-v4.4/esp32s2/index.html)
[![ESP-IDF](https://img.shields.io/badge/ESP--IDF-v4.4-blue.svg)](https://docs.espressif.com/projects/esp-idf/en/release-v4.4/index.html)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

A self-contained, high-performance HTTP/WebSocket Ground Control Station (GCS) component designed to run directly on the ESP32 (specifically optimized for ESP32-S2) quadcopter flight controller stack. It replaces the legacy UDP/CRTP mobile application with a modern, offline-capable single-page web dashboard served straight from the drone's flash memory.

---

## Key Features

- **Embedded Web Server**: Serves a premium responsive telemetry and flight control dashboard using `esp_http_server` directly from the drone's memory.
- **WebSocket Control Connection**: High frequency control and command execution (up to 50 Hz setpoint updates) with bidirectional telemetry streaming.
- **Single-Client Lockout Safety**: Restricts drone flight controls to the first WebSocket connection, granting subsequent clients view-only telemetry to prevent control hijacking.
- **Link Watchdog Safeguard**: Monitors communication latency; automatically triggers an `AUTOLAND` sequence (thrust ramp-down) and disarms if connection is lost for more than 600 ms.
- **Arming Interlock Security**: Requires Attitude alignment ($< 5.0^\circ$), complete IMU sensor calibration, and safe battery levels ($> 3.2\text{ V}$) to transition out of the `DISARMED` state.
- **Command Slew-Rate Limiting**: Clamps roll/pitch/yaw/thrust to safe limits and applies first-order slew filters (e.g., maximum thrust change rate of 20,000 units/second) to prevent motor burnouts or aggressive control spikes.
- **On-board Wifi AP Configuration**: Configure local WPA2 Access Point SSID and password credentials saved to NVS directly from the web panel.

---

## Repository Structure

```
ESP32 GCS/
├── .gitignore               # Build folder, temporary configuration, and IDE files to ignore
├── LICENSE                  # MIT License
├── README.md                # This comprehensive guide
├── implementation_plan.md   # Architectural implementation blueprint
├── docs/
│   ├── design_doc.md        # Technical architecture, WebSocket schemas, and state machine transitions
│   └── flashing_guide.md    # Windows building, configuring, and flashing steps using ESP-IDF
├── src/
│   └── webctrl/             # Main ESP-IDF Component
│       ├── CMakeLists.txt   # Component registration file
│       ├── webctrl.c        # HTTP server, WebSocket loops, timers, watchdog, and controller logic
│       ├── include/
│       │   └── webctrl.h    # Component initialization header
│       └── web/
│           └── index.html   # HTML5/CSS3/JS offline web dashboard
└── tests/
    └── tests_main.c         # C-based mock framework unit tests (executable on host computer)
```

---

## Flight Stack Integration Guide

To integrate this `webctrl` Ground Control Station component into your standard ESP-Drone flight stack, follow these steps:

### 1. Register the Component

Copy the `src/webctrl` folder into your ESP-Drone components or workspace, and update your main application's root `CMakeLists.txt` to include the folder in `PLANE_COMPONENT_DIRS`:

```cmake
set(PLANE_COMPONENT_DIRS 
    "./components/core"
    "./components/drivers"
    # ... other directories ...
    "/path/to/ESP32 GCS/src/webctrl"
)
```

In the main application component's registration (`main/CMakeLists.txt`), add `webctrl` to the requirements list:
```cmake
idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       LDFRAGMENTS linker_fragment.lf
                       REQUIRES webctrl crazyflie)
```

### 2. Expose Stabilizer Telemetry

Declare and implement a telemetry getter in the flight stabilizer loop (`stabilizer.c` / `stabilizer.h`) to share estimated roll, pitch, yaw, and loop frequency:

```c
// In stabilizer.h
void stabilizerGetTelemetry(float *roll, float *pitch, float *yaw, float *thrustRate, float *loopFreq);

// In stabilizer.c
void stabilizerGetTelemetry(float *roll, float *pitch, float *yaw, float *thrustRate, float *loopFreq) {
    // Safely copy state estimation variables to output pointers
    *roll = state.attitude.roll;
    *pitch = state.attitude.pitch;
    *yaw = state.attitude.yaw;
    *thrustRate = setpoint.thrust;
    *loopFreq = stabilizerTaskFreq;
}
```

### 3. Initialize on Main Boot

Modify the main boot application entry point (`main.c`) to import and initialize the GCS module after the core flight stack has finished setting up and waiting for start:

```c
#include "webctrl.h"

static void webctrl_start_task(void *pvParameters)
{
    // Wait for the system task to finish initialization and start
    systemWaitStart();
    vTaskDelay(pdMS_TO_TICKS(100)); // Delay slightly to settle
    
    // Initialize and start the HTTP/WebSocket server and control tasks
    webctrlInit();
    vTaskDelete(NULL);
}

void app_main()
{
    // ... platform and system tasks launched ...
    systemLaunch();
    
    // Launch GCS startup task
    xTaskCreate(webctrl_start_task, "webctrl_start", 2048, NULL, 3, NULL);
}
```

---

## Building and Flashing

Ensure you have your ESP-IDF v4.4 environment loaded:

1. **Set target chip to ESP32-S2**:
   ```bash
   idf.py set-target esp32s2
   ```
2. **Configure build via Menuconfig**:
   ```bash
   idf.py menuconfig
   ```
   Navigate to `ESPDrone Config` -> `app set` and toggle:
   - `Enable WiFi Web Control Ground Station` -> `Yes`
   - `Disable legacy UDP/CRTP vendor communications` -> `Yes` (recommended)
3. **Build the project**:
   ```bash
   idf.py build
   ```
4. **Flash to COM Port** (e.g., `COM3`):
   ```bash
   idf.py -p COM3 flash
   ```
5. **Open Monitor**:
   ```bash
   idf.py -p COM3 monitor
   ```

*Refer to [docs/flashing_guide.md](docs/flashing_guide.md) for full setup troubleshooting.*

---

## Verification & Host-Side Unit Tests

The repository includes a host-side unit test suite that mocks ESP-IDF tasks, networking sockets, NVS memory, and flight controller modules. These tests verify the parser, clamping ranges, safety thresholds, and watchdog state transitions.

### Run Tests locally

Navigate to the repository folder on your terminal:

1. **Compile the test binary**:
   ```bash
   gcc -I"src/webctrl" -o "tests/host_tests" "tests/tests_main.c"
   ```
2. **Execute the test binary**:
   ```bash
   ./tests/host_tests
   ```

You should see output confirming that all tests (clamping bounds, arming limits, slew rates, watchdog autoland) passed successfully.
