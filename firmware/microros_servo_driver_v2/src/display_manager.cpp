#include "display_manager.h"
#include "debug_utils.h"
#include "system_state.h"

DisplayManager g_display;

DisplayManager::DisplayManager() : 
    display_(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET),
    initialized_(false),
    last_update_time_(0),
    current_page_(0) {}

void DisplayManager::begin() {
    DEBUG_INFO("Initializing display manager");
    
    if (!display_.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        DEBUG_ERROR("SSD1306 display allocation failed");
        return;
    }
    
    initialized_ = true;
    display_.clearDisplay();
    display_.setTextSize(1);
    display_.setTextColor(SSD1306_WHITE);
    
    showBootScreen();
    
    DEBUG_INFO("Display manager initialized");
}

void DisplayManager::update() {
    if (!initialized_) return;
    
    unsigned long current_time = millis();
    if (current_time - last_update_time_ >= DISPLAY_UPDATE_INTERVAL) {
        updateDisplay();
        last_update_time_ = current_time;
    }
}

void DisplayManager::showBootScreen() {
    if (!initialized_) return;
    
    display_.clearDisplay();
    display_.setTextSize(2);
    display_.setCursor(0, 0);
    display_.println("LeRemix");
    display_.setTextSize(1);
    display_.setCursor(0, 20);
    display_.println("Servo Driver v2");
    display_.setCursor(0, 35);
    display_.println("Initializing...");
    display_.display();
    
    delay(2000);
}

void DisplayManager::updateDisplay() {
    if (!initialized_) return;
    
    display_.clearDisplay();
    display_.setTextSize(1);
    display_.setCursor(0, 0);
    
    switch (current_page_) {
        case 0:
            showStatusPage();
            break;
        case 1:
            showNetworkPage();
            break;
        case 2:
            showServoPage();
            break;
    }
    
    display_.display();
}

void DisplayManager::showStatusPage() {
    SystemState state = g_system_state.getCurrentState();
    const char* state_names[] = {"DISCONNECTED", "SAFE", "READY"};
    
    display_.println("=== STATUS ===");
    display_.print("State: ");
    display_.println(state_names[state]);
    
    display_.print("ROS: ");
    display_.println(g_system_state.isRosConnected() ? "CONNECTED" : "DISCONNECTED");
    
    display_.print("Servos: ");
    display_.println(g_system_state.areServosEnabled() ? "ENABLED" : "DISABLED");
    
    display_.print("WiFi: ");
    display_.println(WiFi.status() == WL_CONNECTED ? "CONNECTED" : "DISCONNECTED");
    
    if (WiFi.status() == WL_CONNECTED) {
        display_.print("IP: ");
        display_.println(WiFi.localIP());
    }
    
    display_.print("Uptime: ");
    display_.print(millis() / 1000);
    display_.println("s");
}

void DisplayManager::showNetworkPage() {
    display_.println("=== NETWORK ===");
    
    if (WiFi.status() == WL_CONNECTED) {
        display_.print("SSID: ");
        display_.println(WiFi.SSID());
        
        display_.print("IP: ");
        display_.println(WiFi.localIP());
        
        display_.print("RSSI: ");
        display_.print(WiFi.RSSI());
        display_.println(" dBm");
        
        display_.print("Web: http://");
        display_.println(WiFi.localIP());
    } else {
        display_.println("WiFi: DISCONNECTED");
        display_.println("Attempting to");
        display_.println("reconnect...");
    }
}

void DisplayManager::showServoPage() {
    display_.println("=== SERVOS ===");
    
    display_.print("Status: ");
    display_.println(g_system_state.areServosEnabled() ? "ENABLED" : "DISABLED");
    
    display_.print("Base E-Stop: ");
    display_.println(g_system_state.isBaseEmergencyStop() ? "YES" : "NO");
    
    display_.print("Arm E-Stop: ");
    display_.println(g_system_state.isArmEmergencyStop() ? "YES" : "NO");
    
    display_.print("Watchdog: ");
    display_.println(g_system_state.isWatchdogEnabled() ? "ENABLED" : "DISABLED");
    
    unsigned long base_timeout = millis() - g_system_state.getLastBaseCommandTime();
    unsigned long arm_timeout = millis() - g_system_state.getLastArmCommandTime();
    
    display_.print("Base cmd: ");
    display_.print(base_timeout);
    display_.println("ms ago");
    
    display_.print("Arm cmd: ");
    display_.print(arm_timeout);
    display_.println("ms ago");
}

void DisplayManager::nextPage() {
    current_page_ = (current_page_ + 1) % DISPLAY_PAGES;
    updateDisplay();
}

void DisplayManager::showMessage(const String& message, unsigned long duration) {
    if (!initialized_) return;
    
    display_.clearDisplay();
    display_.setCursor(0, 0);
    display_.setTextSize(1);
    display_.println("=== MESSAGE ===");
    display_.println();
    display_.println(message);
    display_.display();
    
    if (duration > 0) {
        delay(duration);
        updateDisplay();
    }
}

void DisplayManager::showError(const String& error) {
    showMessage("ERROR:\n" + error, 3000);
}

void DisplayManager::showInfo(const String& info) {
    showMessage("INFO:\n" + info, 2000);
}