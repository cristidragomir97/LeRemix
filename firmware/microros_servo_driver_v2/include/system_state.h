#pragma once

#include <Arduino.h>

// System state enumeration
enum SystemState {
    STATE_DISCONNECTED,     // Red - Not connected to ROS
    STATE_CONNECTED_SAFE,   // Yellow - Connected but servos disabled
    STATE_READY            // Green - All systems ready
};

// System state manager class
class SystemStateManager {
public:
    SystemStateManager();
    void begin();
    void update();
    
    // State getters/setters
    SystemState getCurrentState() const { return current_state_; }
    bool isRosConnected() const { return ros_connected_; }
    bool areServosEnabled() const { return servos_enabled_; }
    
    void setRosConnected(bool connected);
    void setServosEnabled(bool enabled);
    void toggleServos();
    
    // Watchdog functions
    void updateBaseCommandTime();
    void updateArmCommandTime();
    void enableWatchdog();
    void checkWatchdog();
    
    // Emergency stop states
    bool isBaseEmergencyStop() const { return base_emergency_stop_; }
    bool isArmEmergencyStop() const { return arm_emergency_stop_; }

private:
    SystemState current_state_;
    bool ros_connected_;
    bool servos_enabled_;
    bool watchdog_enabled_;
    
    // Watchdog timing
    unsigned long last_base_cmd_time_;
    unsigned long last_arm_cmd_time_;
    
    // Emergency stop flags
    bool base_emergency_stop_;
    bool arm_emergency_stop_;
    
    void updateSystemState();
};

extern SystemStateManager g_system_state;