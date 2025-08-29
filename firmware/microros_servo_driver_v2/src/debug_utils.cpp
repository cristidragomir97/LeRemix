#include "debug_utils.h"

void DebugUtils::begin() {
#if ENABLE_DEBUG_PRINTS
    Serial.begin(115200);
    while (!Serial && millis() < 2000) {
        delay(10);
    }
    Serial.println("\n=== LeRemix Servo Driver v2 Debug Started ===");
#endif
}

void DebugUtils::print(const char* msg) {
#if ENABLE_DEBUG_PRINTS
    Serial.print("[DEBUG] ");
    Serial.println(msg);
#endif
}

void DebugUtils::printf(const char* format, ...) {
#if ENABLE_DEBUG_PRINTS
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    Serial.print("[DEBUG] ");
    Serial.println(buffer);
#endif
}

void DebugUtils::printCallback(const char* callback_name, const char* details) {
#if ENABLE_DEBUG_PRINTS
    Serial.printf("[CALLBACK] %s: %s\n", callback_name, details);
#endif
}

void DebugUtils::printError(const char* error_msg) {
#if ENABLE_DEBUG_PRINTS
    Serial.print("[ERROR] ");
    Serial.println(error_msg);
#endif
}

void DebugUtils::printWarning(const char* warning_msg) {
#if ENABLE_DEBUG_PRINTS
    Serial.print("[WARNING] ");
    Serial.println(warning_msg);
#endif
}

void DebugUtils::printInfo(const char* info_msg) {
#if ENABLE_DEBUG_PRINTS
    Serial.print("[INFO] ");
    Serial.println(info_msg);
#endif
}