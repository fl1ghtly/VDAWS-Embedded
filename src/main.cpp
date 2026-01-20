/**
 * @file main.cpp
 * @brief ESP32-S3 Drone Sensor and Camera System
 * 
 * This firmware integrates multiple sensors and a camera module for drone applications:
 * - GPS module for location tracking
 * - BMP280 barometric sensor for altitude measurement
 * - OV7670 camera for video streaming and image capture
 * - WiFi access point for wireless control and data transmission
 * 
 * @author Wayne Lai
 * @date December 2025
 * @version 1.0.0
 * @license MIT
 */

#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <WiFi.h>
#include <WebServer.h>
#include <driver/gpio.h>
#include <ArduinoJson.h>

/**
 * @brief Workaround for sensor_t type conflict
 * 
 * Both Adafruit_Sensor and esp_camera libraries define "sensor_t".
 * This macro renames the camera's sensor_t to avoid compilation errors.
 */
#define sensor_t camera_sensor_t
#include "esp_camera.h"
#undef sensor_t
#include "img_converters.h"

// Debug configuration: Set to 1 to echo raw NMEA sentences from GPS
#define ECHO_RAW_NMEA 0

// ============================================================================
// VSYNC DEBOUNCING CONFIGURATION
// ============================================================================
/**
 * @brief VSYNC debouncing delay in microseconds
 * 
 * Prevents spurious VSYNC signals from triggering false frame captures.
 * Adjust based on expected frame rate (e.g., 50us for ~20kHz max signal rate).
 */
#define VSYNC_DEBOUNCE_US 50

// VSYNC stability tracking variables
volatile bool vsync_stable = true;           ///< Flag indicating VSYNC signal stability
volatile unsigned long last_vsync_time = 0;  ///< Timestamp of last VSYNC edge

// ============================================================================
// WIFI CONFIGURATION
// ============================================================================
/**
 * @brief WiFi Access Point credentials
 * 
 * The ESP32-S3 creates its own WiFi network for client connections.
 * Connect to this network to access the web interface and camera stream.
 */
const char* ssid = "Drone-Cam-v2";      ///< WiFi network name (SSID)
const char* password = "12345678";      ///< WiFi password (minimum 8 characters)

/**
 * @brief Web server instance
 * 
 * Listens on port 80 for HTTP requests, serving the web interface
 * and handling camera stream/capture endpoints.
 */
WebServer server(80);

// ============================================================================
// CAMERA PIN CONFIGURATION (OV7670)
// ============================================================================
/**
 * @brief OV7670 camera module pin assignments for ESP32-S3
 * 
 * Pin selection avoids conflicts with GPS UART (GPIO 16/17) and 
 * BMP280 I2C (GPIO 8/9). All pins are on available GPIO on the DevKitC-1.
 * 
 * Hardware connections:
 * - PWDN should be connected to GND (camera always powered)
 * - RESET should be connected to 3.3V (camera always active)
 */

// Power and reset pins (not used in this configuration)
#define PWDN_GPIO_NUM    -1    ///< Power down (connect to GND)
#define RESET_GPIO_NUM   -1    ///< Reset pin (connect to 3.3V)

// Clock and I2C configuration pins
#define XCLK_GPIO_NUM    11    ///< Master clock output to camera
#define SIOD_GPIO_NUM    15    ///< I2C SDA for camera configuration
#define SIOC_GPIO_NUM    18    ///< I2C SCL for camera configuration

// 8-bit parallel data bus (Y2-Y9 correspond to D0-D7)
#define Y9_GPIO_NUM      10    ///< Data bit 7 (MSB)
#define Y8_GPIO_NUM      7     ///< Data bit 6
#define Y7_GPIO_NUM      6     ///< Data bit 5
#define Y6_GPIO_NUM      5     ///< Data bit 4
#define Y5_GPIO_NUM      4     ///< Data bit 3
#define Y4_GPIO_NUM      3     ///< Data bit 2
#define Y3_GPIO_NUM      2     ///< Data bit 1
#define Y2_GPIO_NUM      1     ///< Data bit 0 (LSB)

// Synchronization and timing signals
#define VSYNC_GPIO_NUM   13    ///< Vertical sync signal
#define HREF_GPIO_NUM    14    ///< Horizontal reference signal
#define PCLK_GPIO_NUM    12    ///< Pixel clock input

// ============================================================================
// BMP280 BAROMETRIC SENSOR CONFIGURATION
// ============================================================================
/**
 * @brief BMP280 sensor object for I2C communication
 * 
 * Measures atmospheric pressure and temperature, which can be used to
 * calculate altitude. Typical I2C addresses: 0x76 or 0x77.
 */
Adafruit_BMP280 bmp;

/**
 * @brief I2C pin assignments for BMP280 sensor
 * 
 * Uses the ESP32-S3's Wire interface with explicit pin configuration.
 * GPIO 8 and 9 form the default I2C pair on this board layout.
 */
static const int I2C_SDA_PIN = 8;  ///< I2C data line (Serial Data)
static const int I2C_SCL_PIN = 9;  ///< I2C clock line (Serial Clock)

// ============================================================================
// GPS MODULE CONFIGURATION
// ============================================================================
/**
 * @brief GPS module UART pin assignments
 * 
 * Uses HardwareSerial port 1 for communication with GPS module.
 * Typical GPS modules output NMEA sentences at 9600 baud.
 */
static const int GPS_RX_PIN = 16;      ///< ESP32 RX (connects to GPS TX)
static const int GPS_TX_PIN = 17;      ///< ESP32 TX (connects to GPS RX, rarely used)
static const uint32_t GPS_BAUD = 9600; ///< GPS serial baud rate

/**
 * @brief Hardware serial port for GPS communication
 * 
 * Uses UART1 to avoid conflict with USB serial (UART0).
 */
HardwareSerial gpsSerial(1);

/**
 * @brief TinyGPSPlus object for parsing NMEA sentences
 * 
 * Decodes GPS data including latitude, longitude, altitude, speed, etc.
 */
TinyGPSPlus gps;

bool readGPSLatLng(int* lat, int* lng);

// ============================================================================
// WEB SERVER REQUEST HANDLERS
// ============================================================================

/**
 * @brief Handles HTTP requests to the root path "/"
 * 
 * Generates and serves the main HTML interface page with embedded video
 * stream and navigation links to capture and stream endpoints.
 * 
 * @note This is a simple embedded HTML page. For production, consider
 *       serving static files from SPIFFS or LittleFS.
 */
void handleRoot() {
  String html = "<html><body><h1>ESP32 Drone Camera</h1>";
  html += "<p><a href='/capture'>Capture Image</a></p>";
  html += "<p><a href='/stream'>Live Stream</a></p>";
  html += "<p><img src='/stream' style='width:100%; max-width:640px;'></p>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

/**
 * @brief Handles HTTP requests to "/stream" for live MJPEG video
 * 
 * Continuously captures frames from the camera, converts them to JPEG,
 * and streams them to the client using the MJPEG protocol. This creates
 * a real-time video feed viewable in any web browser.
 * 
 * @details Frame Processing Pipeline:
 *   1. Capture RGB565 frame from camera
 *   2. Convert to JPEG format (quality 85)
 *   3. Send as MJPEG multipart frame
 *   4. Free memory and repeat
 * 
 * @note The stream continues until the client disconnects or an error occurs.
 *       Target frame rate is ~15-20 FPS with 50ms delay between frames.
 */
void handleStream() {
  Serial.println("Stream started");
  WiFiClient client = server.client();
  
  // Send MJPEG HTTP headers
  client.write("HTTP/1.1 200 OK\r\n");
  client.write("Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n");
  
  // Discard first frame (often contains artifacts from initialization)
  camera_fb_t * fb_discard = esp_camera_fb_get();
  if (fb_discard) {
    esp_camera_fb_return(fb_discard);
  }
  delay(50); // Allow camera to stabilize
  
  // Frame rate tracking variables
  unsigned long lastFrameTime = millis();
  int frameCount = 0;
  
  // Main streaming loop - continues while client is connected
  while (client.connected()) {
    // Feed watchdog timer to prevent system reset during long operations
    yield();
    
    // Capture frame from camera
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Frame capture failed");
      delay(100);
      continue; // Retry on failure instead of terminating stream
    }
    
    /**
     * Convert RGB565 frame to JPEG format
     * Quality 85 provides good balance between:
     * - Image quality
     * - Processing time
     * - Bandwidth usage
     */
    uint8_t * jpg_buf = NULL;
    size_t jpg_len = 0;
    bool converted = frame2jpg(fb, 85, &jpg_buf, &jpg_len);
    esp_camera_fb_return(fb); // Return frame buffer to driver
    
    if (!converted) {
      Serial.println("JPEG conversion failed");
      delay(100);
      continue; // Retry conversion on next frame
    }
    
    /**
     * Send MJPEG frame to client
     * MJPEG format uses multipart/x-mixed-replace with boundaries
     * Each frame is a complete JPEG image with its own headers
     */
    client.write("--frame\r\n");
    client.write("Content-Type: image/jpeg\r\n");
    client.printf("Content-Length: %d\r\n\r\n", jpg_len);
    
    // Write JPEG data to client
    size_t written = client.write(jpg_buf, jpg_len);
    if (written != jpg_len) {
      Serial.printf("Write failed: sent %d of %d bytes\n", written, jpg_len);
      free(jpg_buf);
      break; // Client disconnected or buffer full
    }
    
    client.write("\r\n"); // Frame boundary terminator
    free(jpg_buf); // Free allocated JPEG buffer
    
    // Track and report frame rate statistics
    frameCount++;
    if (millis() - lastFrameTime >= 5000) {
      Serial.printf("Stream: %d frames in 5s (%.1f FPS)\n", frameCount, frameCount / 5.0);
      frameCount = 0;
      lastFrameTime = millis();
    }
    
    /**
     * Inter-frame delay controls streaming frame rate
     * 50ms delay ≈ 20 FPS maximum
     * Adjust based on network bandwidth and processing capability
     */
    delay(50);
  }
  
  Serial.println("Stream stopped");
}

/**
 * @brief Handles HTTP requests to "/capture" for single image capture
 * 
 * Captures a single high-quality image from the camera, converts it to
 * JPEG format, and sends it to the client. This endpoint is useful for
 * taking snapshots without the overhead of continuous streaming.
 * 
 * @details Capture Process:
 *   1. Discard first frame (may contain artifacts)
 *   2. Wait for camera to stabilize
 *   3. Capture frame
 *   4. Convert RGB565 to JPEG (quality 80)
 *   5. Send JPEG to client
 *   6. Clean up resources
 * 
 * @note Returns HTTP 500 error if capture or conversion fails
 */
void handleCapture() {
  Serial.println("Start Capture...");
  
  // Discard first frame (often contains initialization artifacts)
  camera_fb_t * fb_discard = esp_camera_fb_get();
  if (fb_discard) {
    esp_camera_fb_return(fb_discard);
  }
  
  delay(100); // Allow camera exposure and settings to stabilize
  
  // Capture frame from camera
  camera_fb_t * fb = NULL;
  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    server.send(500, "text/plain", "Camera capture failed");
    return;
  }
  Serial.printf("Frame captured. Width: %d, Height: %d, Size: %d bytes\n", 
                fb->width, fb->height, fb->len);
  
  /**
   * Convert RGB565 to JPEG with quality 80
   * Higher quality than streaming (85) for better single-shot images
   * Measure conversion time for performance monitoring
   */
  uint8_t * jpg_buf = NULL;
  size_t jpg_len = 0;
  unsigned long start = millis();
  bool converted = frame2jpg(fb, 80, &jpg_buf, &jpg_len);
  unsigned long end = millis();
  Serial.printf("JPEG conversion took %lu ms, size: %d bytes\n", end - start, jpg_len);
  
  esp_camera_fb_return(fb); // Return frame buffer to driver
  
  if (!converted) {
    Serial.println("JPEG conversion failed");
    server.send(500, "text/plain", "JPEG conversion failed");
    return;
  }
  
  /**
   * Send JPEG image to client
   * Uses Content-Disposition: inline to display in browser
   * Alternative: use "attachment" to force download
   */
  WiFiClient client = server.client();
  client.write("HTTP/1.1 200 OK\r\n");
  client.write("Content-Type: image/jpeg\r\n");
  client.write("Content-Disposition: inline; filename=capture.jpg\r\n");
  client.write("Content-Length: ");
  client.print(jpg_len);
  client.write("\r\n\r\n");
  client.write(jpg_buf, jpg_len);
  
  free(jpg_buf); // Free allocated JPEG buffer
  Serial.println("Capture sent successfully");
}

/**
 * @brief Handles HTTP requests to "/sensors" for GPS, Barometer, and IMU sensors
 * 
 * Reads from the GPS sensor for latitude and longitude data, while using the Barometer for altitude.
 * Uses data from the IMU to describe orientation. Also includes a timestamp of when measurement occurred. 
 * Returns all data in JSON format.
 * 
 * @note Returns HTTP 500 error if any Sensor fails
 */
void handleSensors() {
  Serial.println("Start Sensors...");

  JsonDocument doc;
  doc["timestamp"] = millis();

  // Get the position of the device
  JsonObject position = doc["position"].to<JsonObject>();
  int lat, lng;
  bool gpsStatus = readGPSLatLng(&lat, &lng);
  if (gpsStatus) {
    position["latitude"] = lat;
    position["longitude"] = lng;
  } else {
    Serial.println("GPS Sensor read failed");
    server.send(500, "text/plain", "GPS Sensor failed to read");
    return;
  }
  position["altitude"] = bmp.readAltitude();
  
  // TODO implement IMU
  // Get the orientation of the device
  JsonObject rotation = doc["rotation"].to<JsonObject>();
  rotation["rx"] = 0;
  rotation["ry"] = 0;
  rotation["rz"] = 0;
  
  // TODO find fov for OV7670
  doc["fov"] = 45;

  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

/**
 * @brief System initialization and configuration
 * 
 * Performs one-time setup of all hardware components and communication
 * interfaces. This includes:
 * - Serial communication for debugging
 * - I2C bus for BMP280 sensor
 * - Camera module initialization
 * - WiFi access point creation
 * - Web server configuration
 * - GPS UART communication
 * 
 * @note Setup includes a 5-second delay for serial monitor attachment
 */
void setup() {
  // Initialize USB serial for debugging output
  Serial.begin(115200);

  // Wait for Serial Monitor connection (useful for debugging startup)
  delay(5000);
  Serial.println("\n\n--- ESP32-S3 Drone System Starting ---");

  // ========================================================================
  // I2C INITIALIZATION
  // ========================================================================
  /**
   * Initialize I2C bus with explicit pin assignment
   * ESP32-S3 requires explicit pin configuration for Wire interface
   */
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  /**
   * I2C Bus Scanner (COMMENTED OUT)
   * 
   * Uncomment this section to scan for I2C devices on the bus.
   * Useful for debugging I2C connection issues or verifying sensor addresses.
   * Scans addresses 0x01 to 0x7F and reports any responding devices.
   */
  /*
  Serial.println("Scanning I2C bus...");
  byte error, address;
  int nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16) Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
      nDevices++;
    }
    else if (error==4) {
      Serial.print("Unknown error at address 0x");
      if (address<16) Serial.print("0");
      Serial.println(address,HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
  */

  // ========================================================================
  // BMP280 SENSOR INITIALIZATION
  // ========================================================================
  /**
   * Initialize BMP280 barometric pressure sensor
   * Common I2C addresses are 0x76 and 0x77
   * Try 0x76 first (common for GY-BMP280 modules), fallback to 0x77
   */
  Serial.println("Initializing BMP280...");
  unsigned status;
  status = bmp.begin(0x76); // Try primary address
  if (!status) {
    status = bmp.begin(0x77); // Try alternate address
  }
  Serial.println("BMP280 init done.");
  
  // ========================================================================
  // CAMERA PRE-INITIALIZATION AND I2C TEST
  // ========================================================================
  /**
   * Pre-enable camera master clock (XCLK) for I2C communication test
   * The OV7670 requires XCLK to be running for I2C configuration
   */
  Serial.println("Pre-enabling XCLK on GPIO 11...");
  pinMode(XCLK_GPIO_NUM, OUTPUT);
  ledcSetup(7, 20000000, 1); // 20 MHz XCLK using LEDC channel 7
  ledcAttachPin(XCLK_GPIO_NUM, 7);
  ledcWrite(7, 1); // 50% duty cycle
  delay(500); // Allow clock to stabilize
  
  /**
   * Test camera I2C communication
   * OV7670 default I2C address is 0x21 (write) / 0x42 (7-bit address)
   * This verifies the camera is connected and powered correctly
   */
  Serial.println("Testing camera I2C communication...");
  Wire1.begin(SIOD_GPIO_NUM, SIOC_GPIO_NUM);
  Wire1.beginTransmission(0x21); // OV7670 I2C write address
  byte i2c_error = Wire1.endTransmission();
  Serial.printf("Camera I2C test result: %d (0=success)\n", i2c_error);
  Wire1.end();
  
  // Release manual LEDC control before camera driver takes over
  ledcDetachPin(XCLK_GPIO_NUM);
  Serial.println("Handing over to camera driver...");

  // ========================================================================
  // CAMERA DRIVER INITIALIZATION
  // ========================================================================
  /**
   * Configure camera settings structure
   * All camera parameters must be set before calling esp_camera_init()
   */
  camera_config_t config;
  
  // LEDC (PWM) configuration for XCLK generation
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  
  // 8-bit parallel data bus pin assignments
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  
  // Clock and synchronization pins
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  
  // I2C (SCCB) pins for camera configuration
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  
  // Power management pins (not used with OV7670)
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  
  /**
   * Camera timing and format configuration
   * - 12 MHz XCLK provides good balance for OV7670
   * - QVGA (320x240) resolution for reasonable frame rate
   * - RGB565 is native format for OV7670 (no on-sensor compression)
   */
  config.xclk_freq_hz = 12000000;           // Master clock: 12 MHz
  config.frame_size = FRAMESIZE_QVGA;       // 320x240 resolution
  config.pixel_format = PIXFORMAT_RGB565;   // 16-bit RGB format
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY; // Wait for empty frame buffer
  
  /**
   * Memory configuration - use PSRAM for frame buffers
   * ESP32-S3 has external PSRAM which is essential for camera operation
   * Without PSRAM, there isn't enough RAM for frame buffers
   */
  config.fb_location = CAMERA_FB_IN_PSRAM;
  Serial.println("Using PSRAM, QVGA (320x240), RGB565, 10MHz XCLK for OV7670...");
  
  config.jpeg_quality = 10;  // Quality for JPEG conversion (0-63, lower=better)
  config.fb_count = 2;       // Use 2 frame buffers for smoother capture

  /**
   * Initialize camera driver
   * Returns ESP_OK (0) on success, error code otherwise
   */
  Serial.println("Initializing OV7670 Camera Driver...");
  esp_err_t err = esp_camera_init(&config);
  bool cameraReady = false;
  
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    cameraReady = false;
  } else {
    Serial.println("Camera initialized successfully!");
    cameraReady = true;
    
    /**
     * Configure camera sensor parameters
     * These settings improve image quality and stability
     */
    camera_sensor_t * s = esp_camera_sensor_get();
    if (s != NULL) {
      s->set_framesize(s, FRAMESIZE_QVGA); // Confirm frame size
      
      // Image orientation settings
      s->set_vflip(s, 0);      // 0 = no vertical flip, 1 = flip vertically
      s->set_hmirror(s, 0);    // 0 = no horizontal mirror, 1 = mirror
      
      /**
       * Enable automatic exposure and gain control
       * Helps camera adapt to changing lighting conditions
       * May introduce slight artifacts but improves overall quality
       */
      s->set_gain_ctrl(s, 1);      // Enable automatic gain control (AGC)
      s->set_exposure_ctrl(s, 1);  // Enable automatic exposure control (AEC)
      s->set_awb_gain(s, 1);       // Enable auto white balance gain
      s->set_wb_mode(s, 0);        // Auto white balance mode
      
      Serial.println("Camera sensor configured");
    }
    
    /**
     * Note on hardware filtering:
     * Hardware glitch filtering on VSYNC would require ESP-IDF functions
     * not available in Arduino framework. Software debouncing (discarding
     * first frame in streaming) provides adequate protection instead.
     */
    Serial.println("Camera configured with software debouncing");
  }
  
  // ========================================================================
  // WIFI ACCESS POINT INITIALIZATION
  // ========================================================================
  /**
   * Configure ESP32-S3 as WiFi Access Point
   * This creates a standalone network that clients can connect to
   */
  WiFi.mode(WIFI_AP); // Set to Access Point mode (not Station mode)
  WiFi.softAP(ssid, password);
  Serial.print("AP Started. IP: ");
  Serial.println(WiFi.softAPIP()); // Typically 192.168.4.1

  // ========================================================================
  // WEB SERVER INITIALIZATION
  // ========================================================================
  /**
   * Register HTTP request handlers and start web server
   * Each endpoint serves a specific function:
   * - "/" : Main interface page
   * - "/capture" : Single image capture
   * - "/stream" : Continuous MJPEG video stream
   */
  server.on("/", handleRoot);
  server.on("/capture", handleCapture);
  server.on("/stream", handleStream);
  server.on("/sensors", handleSensors);
  server.begin();
  Serial.println("HTTP server started");

  // ========================================================================
  // STARTUP DIAGNOSTICS
  // ========================================================================
  /**
   * Print startup information multiple times for visibility
   * Attempts to re-initialize BMP280 if initial attempt failed
   * Reports status of all major components
   */
  int startupRepeats = 5;
  while (startupRepeats > 0) {
    Serial.println("Hello from UART!");
    Serial.println("ESP32-S3 GPS + BMP280 + OV7670 test starting...");
    
    /**
     * Retry BMP280 initialization with different parameters
     * Some modules identify as BMP280 (ID 0x58) or BME280 (ID 0x60)
     */
    if (!status) {
       status = bmp.begin(0x76, 0x58); // Try with BMP280 chip ID
       if (!status) {
         status = bmp.begin(0x76, 0x60); // Try with BME280 chip ID
       }
       if (!status) {
         status = bmp.begin(0x76); // Try generic initialization
       }
    }

    // Report component status
    if (!status) {
      Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    } else {
      Serial.println("BMP280 sensor found!");
    }

    if (cameraReady) {
      Serial.println("OV7670 Camera Ready!");
    } else {
      Serial.printf("Camera Init Failed! Error: 0x%x\n", err);
    }

    Serial.print("Startup countdown: ");
    Serial.println(startupRepeats);
    delay(1000);
    startupRepeats--;
  }

  // ========================================================================
  // GPS SERIAL INITIALIZATION
  // ========================================================================
  /**
   * Start GPS serial communication on UART1
   * Standard GPS modules use 9600 baud with 8N1 format
   */
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("GPS serial started");
  
  // ========================================================================
  // BMP280 ADVANCED CONFIGURATION
  // ========================================================================
  /**
   * Configure BMP280 sampling and filtering for optimal accuracy
   * These settings balance accuracy, noise reduction, and response time
   */
  if (status) {
    bmp.setSampling(
      Adafruit_BMP280::MODE_NORMAL,     // Continuous measurement mode
      Adafruit_BMP280::SAMPLING_X2,     // 2x temperature oversampling
      Adafruit_BMP280::SAMPLING_X16,    // 16x pressure oversampling (high accuracy)
      Adafruit_BMP280::FILTER_X16,      // 16x IIR filter (smooth readings)
      Adafruit_BMP280::STANDBY_MS_500   // 500ms between measurements
    );
  }
}

/**
 * @brief Reads from the GPS sensor the latitude and longitude
 * 
 * Returns the latitude and longitude into their respective function arguments
 * 
 * @note Also returns a boolean stating whether GPS read was successful
 */
bool readGPSLatLng(int* lat, int* lng) {
  /**
   * Read and parse GPS data from serial port
   * TinyGPSPlus incrementally decodes NMEA sentences
   * Characters are fed one at a time to the GPS parser
   */
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
#if ECHO_RAW_NMEA
    Serial.write(c); // Echo raw NMEA sentences if enabled
#endif
    gps.encode(c); // Feed character to GPS parser
  }

  /**
   * Display GPS location when valid fix is obtained
   * GPS must have clear view of sky and lock onto satellites
   * Typically takes 30-60 seconds for first fix (cold start)
   */
  if (gps.location.isUpdated() && gps.location.isValid()) {
    *lat = gps.location.lat();
    *lng = gps.location.lng();

    return true;
  }

  return false;
}

/**
 * @brief Main program loop
 * 
 * Continuously performs the following tasks:
 * - Handle incoming HTTP requests from web clients
 * - Process GPS data from serial port
 * - Read and display sensor data periodically
 * - Print system health information
 * 
 * @note This function never returns; it runs indefinitely
 */
void loop() {
  // ========================================================================
  // WEB SERVER REQUEST HANDLING
  // ========================================================================
  /**
   * Process any pending HTTP requests
   * Must be called frequently to maintain responsive web interface
   */
  server.handleClient();
  
  // ========================================================================
  // SYSTEM HEARTBEAT
  // ========================================================================
  /**
   * Periodic heartbeat message and memory usage report
   * Helps verify system is running and monitor resource usage
   * Prints every 5 seconds
   */
  static unsigned long lastHeartbeat = 0;
  if (millis() - lastHeartbeat > 5000) {
     lastHeartbeat = millis();
     Serial.println("Heartbeat: Loop is running...");
     Serial.printf("Free Heap: %d, Free PSRAM: %d\n", 
                   ESP.getFreeHeap(), ESP.getFreePsram());
  }

  // ========================================================================
  // GPS DATA PROCESSING
  // ========================================================================
  static unsigned long lastGPSPrint = 0;
  if (millis() - lastGPSPrint >= 2000) {
    lastGPSPrint = millis();
    
    int lat, lng;
    bool gpsStatus = readGPSLatLng(&lat, &lng);
    if (gpsStatus) {
      Serial.print("Latitude: ");
      Serial.println(lat, 6); // 6 decimal places
      Serial.print("Longitude: ");
      Serial.println(lng, 6);
    } else {
      Serial.println("ERROR: GPS Read failed");
    }
  }

  // ========================================================================
  // BMP280 SENSOR DATA LOGGING
  // ========================================================================
  /**
   * Read and display barometric sensor data periodically
   * Updates every 2 seconds to avoid flooding serial output
   * 
   * Measurements include:
   * - Temperature in degrees Celsius
   * - Atmospheric pressure in Pascals
   * - Calculated altitude in meters (based on standard sea level pressure)
   */
  static unsigned long lastBmpPrint = 0;
  if (millis() - lastBmpPrint >= 2000) {
    lastBmpPrint = millis();
    
    Serial.print("Temperature = ");
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");

    Serial.print("Pressure = ");
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");

    /**
     * Altitude calculation uses barometric formula
     * Assumes standard sea level pressure of 1013.25 hPa
     * For more accurate altitude, use local sea level pressure
     */
    Serial.print("Approx altitude = ");
    Serial.print(bmp.readAltitude(1013.25));
    Serial.println(" m");
    
    Serial.println(); // Blank line for readability
  }
}
