#pragma once

#include <Arduino.h>
#include "config.h"

// Debug printing functions
class DebugUtils {
public:
    static void begin();
    static void print(const char* msg);
    static void printf(const char* format, ...);
    static void printCallback(const char* callback_name, const char* details);
    static void printError(const char* error_msg);
    static void printWarning(const char* warning_msg);
    static void printInfo(const char* info_msg);
};

// Convenience macros
#if ENABLE_DEBUG_PRINTS
    #define DEBUG_PRINT(msg) DebugUtils::print(msg)
    #define DEBUG_PRINTF(...) DebugUtils::printf(__VA_ARGS__)
    #define DEBUG_CALLBACK(name, details) DebugUtils::printCallback(name, details)
    #define DEBUG_ERROR(msg) DebugUtils::printError(msg)
    #define DEBUG_WARNING(msg) DebugUtils::printWarning(msg)
    #define DEBUG_INFO(msg) DebugUtils::printInfo(msg)
#else
    #define DEBUG_PRINT(msg)
    #define DEBUG_PRINTF(...)
    #define DEBUG_CALLBACK(name, details)
    #define DEBUG_ERROR(msg)
    #define DEBUG_WARNING(msg)
    #define DEBUG_INFO(msg)
#endif