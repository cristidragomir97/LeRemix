#include <Arduino.h>
#include "config.h"
#include "debug_utils.h"
#include "system_state.h"
#include "neopixel_manager.h"
#include "button_manager.h"
#include "servo_manager.h"
#include "display_manager.h"
#include "microros_manager.h"

void setup() {
    // Initialize debug output first
    DebugUtils::begin();
    DEBUG_INFO("=== LeRemix Servo Driver v2 Starting ===");
    
    // Initialize system state manager
    g_system_state.begin();
    
    // Initialize hardware components
    g_neopixel.begin();
    g_button.begin();
    g_display.begin();
    
    // Initialize servo system
    g_servo.begin();
    
    // Show servo initialization result
    if (g_servo.isInitialized()) {
        g_display.showInfo("Servos OK\nStarting ROS...");
        DEBUG_INFO("Servos initialized successfully");
    } else {
        g_display.showError("Servo Init Failed\nCheck connections");
        DEBUG_ERROR("Servo initialization failed - continuing without servos");
    }
    
    // Initialize micro-ROS communication
    g_microros.begin();
    
    // Show final status
    if (g_microros.isInitialized()) {
        g_display.showInfo("All Systems Ready\nROS Connected");
        DEBUG_INFO("=== All systems initialized successfully ===");
    } else {
        g_display.showError("ROS Init Failed\nCheck agent");
        DEBUG_ERROR("Micro-ROS initialization failed - operating in offline mode");
    }
    
    // Final system state update
    g_system_state.update();
    
    DEBUG_INFO("Setup complete - entering main loop");
}

void loop() {
    // Update all system components
    g_system_state.update();
    g_button.update();
    g_neopixel.update();
    g_display.update();
    // No logger update needed
    
    // Update servo and micro-ROS systems
    g_servo.update();
    g_microros.update();
    
    // Handle emergency stop conditions
    if (g_system_state.isBaseEmergencyStop() || g_system_state.isArmEmergencyStop()) {
        // Emergency stop is automatically handled by the servo manager
        // based on system state, no additional action needed here
    }
    
    // Small delay to prevent excessive CPU usage
    delay(1);
}