#pragma once

// Debug and control defines
#define ENABLE_SERVO_CONTROL 1      // Set to 0 to disable servo functionality
#define ENABLE_DEBUG_PRINTS 1       // Set to 0 to disable debug prints
#define ENABLE_WEB_INTERFACE 1      // Set to 0 to disable web interface

// Display configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

// GPIO Configuration
#define S_RXD 18                    // Servo communication RX
#define S_TXD 19                    // Servo communication TX
#define S_SCL 22                    // Display I2C clock
#define S_SDA 21                    // Display I2C data
#define RGB_LED 23                  // NeoPixel data pin
#define NUMPIXELS 10                // Number of NeoPixels
#define EN_BUTTON_PIN 0             // Boot button (EN)

// Timing Configuration
#define LONG_PRESS_TIME 3000        // Button long press time (ms)
#define CONTROL_PERIOD_MS 5         // Control loop period (200Hz)
#define WATCHDOG_TIMEOUT_MS 500     // Servo safety timeout
#define WEB_UPDATE_PERIOD_MS 100    // Web interface update rate

// Network Configuration - CHANGE THESE!
#define WIFI_SSID "your_wifi_ssid"
#define WIFI_PASSWORD "your_wifi_password"
#define MICROROS_AGENT_IP "192.168.1.100"  // Your ROS2 machine IP
#define MICROROS_AGENT_PORT 8888

// ROS Topic Configuration
#define BASE_CMD_TOPIC "/esp32/base_cmd"
#define ARM_CMD_TOPIC "/esp32/arm_cmd"
#define JOINT_STATE_TOPIC "/esp32/joint_states"

// Servo Configuration
#define BASE_SERVO_COUNT 3
#define ARM_SERVO_COUNT 7
#define TOTAL_SERVO_COUNT (BASE_SERVO_COUNT + ARM_SERVO_COUNT)

// Hardware constants
#define VEL_TO_SERVO_UNIT 13037     // Conversion factor from rad/s to servo speed units
#define COUNTS_PER_REV 4096         // Encoder counts per revolution

// Servo calibration data
struct ServoCalibration {
    uint8_t servo_id;
    int16_t min_position;
    int16_t max_position;
    int16_t relaxed_position;
};

// Joint calibration data measured from physical robot
extern const ServoCalibration ARM_CALIBRATION[];
extern const int ARM_CALIBRATION_COUNT;
extern const uint8_t BASE_SERVO_IDS[BASE_SERVO_COUNT];
extern const uint8_t ARM_SERVO_IDS[ARM_SERVO_COUNT];