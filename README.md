# ESP32-S3 Drone Sensor & Camera System

A comprehensive embedded system for drone applications built on the ESP32-S3 platform, integrating GPS tracking, barometric altitude sensing, and live camera streaming capabilities.

## 📋 Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [Pin Configuration](#pin-configuration)
- [Software Dependencies](#software-dependencies)
- [Installation](#installation)
- [Usage](#usage)
- [API Endpoints](#api-endpoints)
- [Configuration](#configuration)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [License](#license)

## 🎯 Overview

This project implements a multi-sensor drone system using the ESP32-S3 microcontroller. It provides real-time GPS positioning, barometric altitude measurement, and wireless camera streaming through a web interface. The system creates its own WiFi access point for easy connectivity and control.

## ✨ Features

- **GPS Tracking**: Real-time location tracking using TinyGPSPlus library
- **Altitude Sensing**: Barometric pressure and altitude measurement via BMP280 sensor
- **Live Camera Streaming**: MJPEG video streaming from OV7670 camera module
- **Image Capture**: On-demand image capture with JPEG conversion
- **WiFi Access Point**: Self-hosted WiFi network for wireless control
- **Web Interface**: Browser-based control and monitoring
- **PSRAM Utilization**: Efficient memory management for frame buffers
- **Serial Debugging**: Comprehensive logging and diagnostics

## 🔧 Hardware Requirements

### Main Components

| Component | Model | Purpose |
|-----------|-------|---------|
| Microcontroller | ESP32-S3 DevKitC-1 | Main processor with WiFi/BLE |
| GPS Module | Generic UART GPS | Location tracking |
| Barometric Sensor | BMP280 (GY-BMP280) | Altitude and pressure measurement |
| Camera | OV7670 | Video capture and streaming |

### Power Requirements

- **Voltage**: 3.3V for all sensors and camera
- **Current**: Ensure power supply can handle ~500mA peak during WiFi transmission

## 📌 Pin Configuration

### GPS Module (UART)
```
ESP32-S3 GPIO 16 (RX) → GPS TX
ESP32-S3 GPIO 17 (TX) → GPS RX
```

### BMP280 Sensor (I2C)
```
ESP32-S3 GPIO 8 (SDA) → BMP280 SDA
ESP32-S3 GPIO 9 (SCL) → BMP280 SCL
```

### OV7670 Camera
```
ESP32-S3 GPIO  1 → Y2 (D0)
ESP32-S3 GPIO  2 → Y3 (D1)
ESP32-S3 GPIO  3 → Y4 (D2)
ESP32-S3 GPIO  4 → Y5 (D3)
ESP32-S3 GPIO  5 → Y6 (D4)
ESP32-S3 GPIO  6 → Y7 (D5)
ESP32-S3 GPIO  7 → Y8 (D6)
ESP32-S3 GPIO 10 → Y9 (D7)
ESP32-S3 GPIO 11 → XCLK
ESP32-S3 GPIO 12 → PCLK
ESP32-S3 GPIO 13 → VSYNC
ESP32-S3 GPIO 14 → HREF
ESP32-S3 GPIO 15 → SIOD (SDA)
ESP32-S3 GPIO 18 → SIOC (SCL)
```

**Note**: PWDN and RESET should be connected to GND and 3.3V respectively.

## 📦 Software Dependencies

### PlatformIO Libraries

- **TinyGPSPlus** - GPS data parsing
- **Adafruit BMP280 Library** - Barometric sensor interface
- **Adafruit Unified Sensor** - Sensor abstraction layer
- **Adafruit BusIO** - I2C/SPI communication
- **esp32-camera** - Camera driver for ESP32

### Arduino Framework Libraries

- WiFi
- WebServer
- Wire (I2C)
- SPI

## 🚀 Installation

### Prerequisites

1. Install [PlatformIO](https://platformio.org/install) IDE or PlatformIO Core
2. Install USB-to-Serial drivers for ESP32-S3

### Setup Steps

1. Clone the repository:
   ```bash
   git clone https://github.com/LaiWeiQuan/VDAWS-Embedded.git
   cd VDAWS-Embedded/Drone
   ```

2. Open the project in PlatformIO

3. Build the project:
   ```bash
   pio run
   ```

4. Upload to ESP32-S3:
   ```bash
   pio run --target upload
   ```

5. Monitor serial output:
   ```bash
   pio device monitor
   ```

## 💡 Usage

### Initial Setup

1. Power on the ESP32-S3 device
2. Wait for the system to initialize (approximately 5 seconds)
3. Connect to the WiFi access point:
   - **SSID**: `Drone-Cam-v2`
   - **Password**: `12345678`
4. Open a web browser and navigate to: `http://192.168.4.1`

### Web Interface

The web interface provides three main functions:

1. **Home Page** (`/`) - Overview and navigation
2. **Live Stream** (`/stream`) - Real-time MJPEG video stream
3. **Capture Image** (`/capture`) - Take a single photo

### Serial Monitor Output

The system provides detailed diagnostics via serial monitor at 115200 baud:

- GPS coordinates (when fix is acquired)
- Temperature, pressure, and altitude readings
- Camera status and frame rate statistics
- Memory usage (heap and PSRAM)
- System heartbeat every 5 seconds

## 🌐 API Endpoints

### GET `/`
Returns the main HTML interface with embedded video stream.

**Response**: `text/html`

---

### GET `/stream`
Provides continuous MJPEG stream from the camera.

**Response**: `multipart/x-mixed-replace` with JPEG frames

**Frame Rate**: ~15-20 FPS

**Resolution**: 320x240 (QVGA)

---

### GET `/capture`
Captures and returns a single JPEG image.

**Response**: `image/jpeg`

**Quality**: 80% JPEG compression

## ⚙️ Configuration

### WiFi Settings

Edit in `main.cpp`:
```cpp
const char* ssid = "Drone-Cam-v2";      // Change AP name
const char* password = "12345678";       // Change password (min 8 chars)
```

### GPS Baud Rate

Default is 9600. Adjust if your GPS module uses a different rate:
```cpp
static const uint32_t GPS_BAUD = 9600;
```

### Camera Settings

Modify camera configuration in `setup()`:
```cpp
config.frame_size = FRAMESIZE_QVGA;     // Options: QVGA, VGA, etc.
config.xclk_freq_hz = 12000000;         // Clock frequency (10-20 MHz)
config.jpeg_quality = 10;               // 0-63, lower = better quality
```

### BMP280 I2C Address

Default attempts 0x76, then 0x77:
```cpp
status = bmp.begin(0x76);  // Or 0x77 depending on your module
```

## 🔍 Troubleshooting

### Camera Issues

**Problem**: Camera initialization fails
- Check all pin connections, especially XCLK, PCLK, VSYNC
- Verify 3.3V power supply is stable
- Ensure PWDN is grounded and RESET is high

**Problem**: Image quality is poor or has artifacts
- Adjust JPEG quality setting (lower number = better quality)
- Check lighting conditions
- Verify camera lens is focused

### GPS Issues

**Problem**: No GPS fix
- Ensure clear view of sky (GPS needs satellites)
- Check TX/RX connections (they should be crossed)
- Verify baud rate matches GPS module

### BMP280 Issues

**Problem**: Sensor not detected
- Verify I2C connections (SDA to GPIO 8, SCL to GPIO 9)
- Check I2C address (try both 0x76 and 0x77)
- Ensure pull-up resistors are present on SDA/SCL lines

### WiFi Issues

**Problem**: Cannot connect to access point
- Verify ESP32-S3 has initialized (check serial monitor)
- Ensure password is at least 8 characters
- Try forgetting and reconnecting to the network

### Memory Issues

**Problem**: Out of memory errors
- PSRAM must be enabled (check `platformio.ini`)
- Reduce frame buffer count
- Lower camera resolution

## 🤝 Contributing

Contributions are welcome! Please follow these guidelines:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

### Code Style

- Use descriptive variable and function names
- Comment complex logic
- Follow existing indentation and formatting
- Test thoroughly before submitting

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](../LICENSE) file for details.

Copyright (c) 2025 James Nguyen

## 🙏 Acknowledgments

- Adafruit for sensor libraries
- TinyGPS++ by Mikal Hart
- ESP32 Arduino Core team
- PlatformIO team

## 📞 Support

For issues, questions, or contributions, please open an issue on GitHub or contact the development team.

---

**Project Status**: Active Development

**Last Updated**: December 2025

**Version**: 1.0.0
