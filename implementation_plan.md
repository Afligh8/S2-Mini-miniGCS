# Implementation Plan - ESP32 GCS GitHub Repository

This plan details the steps to package the ESP32 Ground Control Station (GCS) web controller component, host-side unit tests, and comprehensive documentation into a self-contained, clean GitHub repository folder named `ESP32 GCS`.

---

## User Review Required

> [!IMPORTANT]
> The folder `ESP32 GCS` will be created in the workspace root (`c:\Users\aaasi\Documents\Antigravity\ESP-Drone-main\ESP32 GCS`). This directory is fully structured as a repository ready to be initialized as a Git repo (`git init`) and pushed to your organization.

> [!TIP]
> The source code files (`webctrl.c`, `webctrl.h`, `CMakeLists.txt`, `index.html`) and host-side tests (`tests_main.c`) will be migrated into structured subfolders (`src/` and `tests/`) inside the repository, leaving the original files in `OFFLINE/` untouched for reference.

---

## Open Questions

> [!NOTE]
> None. The repo structure is designed following GitHub best practices.

---

## Proposed Changes

### Repository Structure

```
ESP32 GCS/
├── .gitignore
├── LICENSE
├── README.md
├── docs/
│   ├── design_doc.md
│   └── flashing_guide.md
├── src/
│   └── webctrl/
│       ├── CMakeLists.txt
│       ├── webctrl.c
│       ├── include/
│       │   └── webctrl.h
│       └── web/
│           └── index.html
└── tests/
    └── tests_main.c
```

### 1. Repository Configuration Files

#### [NEW] [.gitignore](file:///c:/Users/aaasi/Documents/Antigravity/ESP-Drone-main/ESP32%20GCS/.gitignore)
*   Ignore compiler outputs, build directories (e.g. ESP-IDF build directory `build/`), local unit test binaries (`host_tests`, `*.exe`, `*.o`), and IDE settings (`.vscode`, `.idea`).

#### [NEW] [LICENSE](file:///c:/Users/aaasi/Documents/Antigravity/ESP-Drone-main/ESP32%20GCS/LICENSE)
*   Create an open-source MIT License file.

#### [NEW] [README.md](file:///c:/Users/aaasi/Documents/Antigravity/ESP-Drone-main/ESP32%20GCS/README.md)
*   A comprehensive, beautiful README explaining the Ground Control Station (GCS) architecture, web control UI layout, configuration, telemetry stream, watchdog mechanism, slew-rate clamping, host-side testing, and step-by-step instructions on how to integrate the `webctrl` component into the standard ESP-Drone flight stack.

---

### 2. Documentation

#### [NEW] [design_doc.md](file:///c:/Users/aaasi/Documents/Antigravity/ESP-Drone-main/ESP32%20GCS/docs/design_doc.md)
*   Copy and polish the GCS Architecture Design Document (detailing WebSocket schema, state machines, and concurrency).

#### [NEW] [flashing_guide.md](file:///c:/Users/aaasi/Documents/Antigravity/ESP-Drone-main/ESP32%20GCS/docs/flashing_guide.md)
*   Copy and polish the Flashing Guide (detailing bootloader mode, target configuration, compiling, and serial monitoring).

---

### 3. Source Code & Tests Migration

#### [NEW] [CMakeLists.txt](file:///c:/Users/aaasi/Documents/Antigravity/ESP-Drone-main/ESP32%20GCS/src/webctrl/CMakeLists.txt)
*   Component registration file, updated to reference `webctrl.c` and embed `web/index.html`.

#### [NEW] [webctrl.c](file:///c:/Users/aaasi/Documents/Antigravity/ESP-Drone-main/ESP32%20GCS/src/webctrl/webctrl.c)
*   The C implementation of the GCS HTTP & WebSocket server.

#### [NEW] [webctrl.h](file:///c:/Users/aaasi/Documents/Antigravity/ESP-Drone-main/ESP32%20GCS/src/webctrl/include/webctrl.h)
*   Header file declaring `webctrlInit()`.

#### [NEW] [index.html](file:///c:/Users/aaasi/Documents/Antigravity/ESP-Drone-main/ESP32%20GCS/src/webctrl/web/index.html)
*   The premium HTML5/JS single-page offline dashboard.

#### [NEW] [tests_main.c](file:///c:/Users/aaasi/Documents/Antigravity/ESP-Drone-main/ESP32%20GCS/tests/tests_main.c)
*   The mock-based host-side test suite, adjusted to include `#include "../src/webctrl/webctrl.c"` rather than the old local path.

---

## Verification Plan

### Automated Tests
*   Compile and execute the host unit tests inside the new directory:
    ```bash
    gcc -I"ESP32 GCS/src/webctrl" -o "ESP32 GCS/tests/host_tests" "ESP32 GCS/tests/tests_main.c"
    ./"ESP32 GCS/tests/host_tests"
    ```
*   Verify that all test cases (clamping, state transitions, watchdog timer, arming interlocks) execute successfully.
