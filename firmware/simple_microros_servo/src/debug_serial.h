#ifndef DEBUG_SERIAL_H
#define DEBUG_SERIAL_H

#include <Arduino.h>

// Debug timing macros for Serial1 with timestamps
#define DEBUG_TIMESTAMP() (millis())

#define DEBUG_LOG(msg) \
    Serial1.printf("[%lu] %s\n", DEBUG_TIMESTAMP(), msg)

#define DEBUG_LOG_F(format, ...) \
    Serial1.printf("[%lu] " format "\n", DEBUG_TIMESTAMP(), ##__VA_ARGS__)

// Specific debug messages for callback and motor events
#define DEBUG_CALLBACK_START(callback_name) \
    DEBUG_LOG_F(">>> %s CALLBACK START", callback_name)

#define DEBUG_CALLBACK_END(callback_name) \
    DEBUG_LOG_F("<<< %s CALLBACK END", callback_name)

#define DEBUG_MOTOR_COMMAND(servo_type, servo_id, value) \
    DEBUG_LOG_F("MOTOR CMD: %s servo %d -> %.4f", servo_type, servo_id, value)

#define DEBUG_MOTOR_SENT(servo_type, servo_id, raw_value) \
    DEBUG_LOG_F("MOTOR SENT: %s servo %d -> %d", servo_type, servo_id, raw_value)

#endif // DEBUG_SERIAL_H