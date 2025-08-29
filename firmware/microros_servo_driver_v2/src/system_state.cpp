#include "system_state.h"
#include "debug_utils.h"
#include "neopixel_manager.h"

SystemStateManager g_system_state;

SystemStateManager::SystemStateManager() :
    current_state_(STATE_DISCONNECTED),
    ros_connected_(false),
    servos_enabled_(false),  // Default DISABLED for safety
    watchdog_enabled_(false),
    last_base_cmd_time_(0),
    last_arm_cmd_time_(0),
    base_emergency_stop_(false),
    arm_emergency_stop_(false) {}

void SystemStateManager::begin() {
    DEBUG_INFO("Initializing system state manager");
    updateSystemState();
}

void SystemStateManager::update() {
    checkWatchdog();
    updateSystemState();
}

void SystemStateManager::setRosConnected(bool connected) {
    if (ros_connected_ != connected) {
        ros_connected_ = connected;
        if (connected) {
            DEBUG_INFO("ROS connection established");
            } else {
            DEBUG_WARNING("ROS connection lost");
            }
        updateSystemState();
    }
}

void SystemStateManager::setServosEnabled(bool enabled) {
    if (servos_enabled_ != enabled) {
        servos_enabled_ = enabled;
        if (enabled) {
            DEBUG_INFO("Servos enabled by system");
            } else {
            DEBUG_INFO("Servos disabled by system");
            }
        updateSystemState();
    }
}

void SystemStateManager::toggleServos() {
    setServosEnabled(!servos_enabled_);
    
    if (servos_enabled_) {
        } else {
        }
}

void SystemStateManager::updateBaseCommandTime() {
    last_base_cmd_time_ = millis();
    base_emergency_stop_ = false;
    
    // Mark ROS as connected when receiving commands
    if (!ros_connected_) {
        setRosConnected(true);
    }
}

void SystemStateManager::updateArmCommandTime() {
    last_arm_cmd_time_ = millis();
    
    // Re-enable torque if recovering from emergency stop
    if (arm_emergency_stop_) {
        DEBUG_INFO("Recovering from arm emergency stop");
        arm_emergency_stop_ = false;
    }
    
    // Mark ROS as connected when receiving commands
    if (!ros_connected_) {
        setRosConnected(true);
    }
}

void SystemStateManager::enableWatchdog() {
    watchdog_enabled_ = true;
    last_base_cmd_time_ = millis();
    last_arm_cmd_time_ = millis();
    DEBUG_INFO("Watchdog enabled");
}

void SystemStateManager::checkWatchdog() {
    if (!watchdog_enabled_) return;
    
    unsigned long current_time = millis();
    
    // Check base command timeout
    if (current_time - last_base_cmd_time_ > WATCHDOG_TIMEOUT_MS && !base_emergency_stop_) {
        base_emergency_stop_ = true;
        DEBUG_WARNING("WATCHDOG: Base emergency stop triggered");
        }
    
    // Check arm command timeout  
    if (current_time - last_arm_cmd_time_ > WATCHDOG_TIMEOUT_MS && !arm_emergency_stop_) {
        arm_emergency_stop_ = true;
        DEBUG_WARNING("WATCHDOG: Arm emergency stop triggered");
        }
    
    // Check for ROS disconnection (extended timeout)
    if (ros_connected_ && 
        (current_time - last_base_cmd_time_ > WATCHDOG_TIMEOUT_MS * 2) && 
        (current_time - last_arm_cmd_time_ > WATCHDOG_TIMEOUT_MS * 2)) {
        setRosConnected(false);
    }
}

void SystemStateManager::updateSystemState() {
    SystemState new_state;
    
    if (!ros_connected_) {
        new_state = STATE_DISCONNECTED;
    } else if (!servos_enabled_) {
        new_state = STATE_CONNECTED_SAFE;
    } else {
        new_state = STATE_READY;
    }
    
    if (new_state != current_state_) {
        current_state_ = new_state;
        g_neopixel.updateStatus(current_state_);
        
        const char* state_names[] = {"DISCONNECTED", "CONNECTED_SAFE", "READY"};
        DEBUG_PRINTF("System state changed to: %s", state_names[current_state_]);
        }
}