# ESP-Drone Flashing Guide

This guide describes how to build, configure, and flash the custom web control firmware onto the ESP32 quadcopter using the ESP-IDF v4.4 environment on Windows.

---

## 1. Prerequisites

Before starting, ensure you have:
1.  **USB-to-UART Bridge Drivers**: The drone typically uses a CP210x or CH340 USB-to-serial chip. Install the appropriate driver for your Windows OS so a COM port is assigned when plugged in.
2.  **ESP-IDF v4.4**: The ESP-Drone flight stack requires ESP-IDF version 4.4. Set this up using the [Espressif Windows Installer](https://docs.espressif.com/projects/esp-idf/en/release-v4.4/esp32/get-started/windows-setup.html).
3.  **USB Cable**: A high-quality micro-USB cable capable of both data transfer and power.

---

## 2. Step-by-Step Flashing Procedure

### Step 2.1: Open the ESP-IDF Command Prompt
Open the **ESP-IDF 4.4 CMD** or **ESP-IDF 4.4 PowerShell** terminal from your Windows Start Menu. This terminal has the path variables (`idf.py`, toolchains, python tools) automatically loaded.

### Step 2.2: Navigate to the Firmware Folder
In your terminal, navigate to the firmware subdirectory:
```powershell
cd "c:\Users\aaasi\Documents\Antigravity\ESP-Drone-main\ESP-Drone-main\Firmware\esp-drone"
```

### Step 2.3: Set the Target Chip (ESP32-S2)
Before configuring and building, set the compiler target to match the ESP32-S2 chip onboard the drone:
```powershell
idf.py set-target esp32s2
```

### Step 2.4: Configure the Build Options
Launch the text-based configuration menu to enable web control:
```powershell
idf.py menuconfig
```
1.  Use the arrow keys to navigate to **ESPDrone Config** -> **app set**.
2.  Highlight **Enable WiFi Web Control Ground Station** and press `Y` to enable.
3.  Highlight **Disable legacy UDP/CRTP vendor communications** and press `Y` (recommended to free up sockets).
4.  Highlight **Enable on-target motor-safe integration test mode** and press `Y`/`N`.
    *   *Note: Set to `Y` for your first flash. This runs the full controller and websocket but disables the physical motors for safe desk testing.*
5.  Press `S` to save your configuration, then `Q` to exit.

### Step 2.5: Build the Project
Compile the bootloader, partition table, and application binaries:
```powershell
idf.py build
```
This generates the flash binaries in the `build/` subdirectory.

### Step 2.6: Connect the Drone and Identify COM Port
1.  Disconnect the drone's flight battery (for safety).
2.  Connect the drone to your PC using the USB cable.
3.  Open Windows **Device Manager** (press `Win + X` -> select **Device Manager**).
4.  Expand **Ports (COM & LPT)** and identify the COM port number assigned to the USB-to-UART bridge (e.g., `COM3` or `COM4`).

### Step 2.7: Flash the Firmware
Flash the built binaries to the drone by specifying the COM port:
```powershell
idf.py -p COM3 flash
```
*(Replace `COM3` with the actual COM port identified in Step 2.6)*.

### Step 2.8: Monitor Output
Start the serial monitor to view debugging logs from the drone's bootloader and flight stack:
```powershell
idf.py -p COM3 monitor
```
*(Press `Ctrl + ]` to exit the monitor).*

---

## 3. Manual Bootloader Mode (Troubleshooting)

Most ESP32 development boards use an auto-reset circuit (DTR/RTS) to enter flash mode automatically. If you encounter the following error during flash:
`A fatal error occurred: Failed to connect to ESP32: Timed out waiting for packet header`

You must manually trigger bootloader download mode:
1.  Locate the physical **BOOT** (or **GPIO0**) button and the **EN** (or **RST**) button on the drone motherboard.
2.  Press and hold the **BOOT** button.
3.  Press and release the **EN**/**RST** button.
4.  Release the **BOOT** button.
5.  Run the flash command again in your terminal: `idf.py -p COMx flash`.

---

## 4. Post-Flash Verification

1.  Disconnect the USB cable.
2.  Plug in the flight battery. The onboard LEDs will initiate self-test flashes.
3.  Scan for WiFi networks on your phone/PC. Look for **ESP-DRONE_XXXXXX** (where `XXXXXX` represents the MAC address).
4.  Connect using the default password `12345678`.
5.  Open your browser and navigate to `http://192.168.43.42` to load the Ground Control dashboard.
