#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels, 32 as default.
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

// GPIO 18 - S_RXD, GPIO 19 - S_TXD, as default.
#define S_RXD 18
#define S_TXD 19

// the IIC used to control OLED screen.
// GPIO 21 - S_SDA, GPIO 22 - S_SCL, as default.
#define S_SCL 22
#define S_SDA 21

// the GPIO used to control RGB LEDs.
// GPIO 23, as default.
#define RGB_LED   23
#define NUMPIXELS 10

// Servo calibration data based on physical measurements
// Each servo has physical limits and relaxed positions measured in encoder counts (0-4095)
struct ServoCalibration {
  uint8_t servo_id;
  int16_t min_position;    // Full range minimum
  int16_t max_position;    // Full range maximum
  int16_t relaxed_position; // Relaxed/neutral position
};

// Joint calibration data measured from physical robot
const ServoCalibration ARM_CALIBRATION[] = {
  {4, 1024, 3072, 2048},   // Joint 1 (pan): full left to full right, relaxed center
  {5, 880, 3072, 1024},    // Joint 2 (shoulder): full up to full down, relaxed down
  {6, 2024, 3880, 3880},   // Joint 3 (elbow): full up to full down, relaxed down
  {7, 922, 3072, 909},     // Joint 4 (wrist tilt): full up to full down, relaxed up
  {8, 1372, 4096, 2048},   // Joint 5 (wrist rotation): full left to full right, relaxed center
  {9, 2048, 3446, 2048},   // Joint 6 (gripper): closed to open, relaxed center
  {11, 1024, 2611, 2048}   // Camera tilt: full down to full up, relaxed center
};

const int ARM_CALIBRATION_COUNT = sizeof(ARM_CALIBRATION) / sizeof(ServoCalibration);