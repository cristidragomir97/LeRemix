#include "neopixel_manager.h"
#include "debug_utils.h"

NeoPixelManager g_neopixel;

NeoPixelManager::NeoPixelManager() : 
    pixels_(NUMPIXELS, RGB_LED, NEO_GRB + NEO_KHZ800),
    last_state_(STATE_DISCONNECTED) {}

void NeoPixelManager::begin() {
    DEBUG_INFO("Initializing NeoPixel manager");
    
    pixels_.begin();
    pixels_.clear();
    pixels_.setBrightness(50);  // Medium brightness
    pixels_.show();
    
    // Initial state indication
    updateStatus(STATE_DISCONNECTED);
    
    DEBUG_INFO("NeoPixel manager initialized");
}

void NeoPixelManager::update() {
    // NeoPixel updates are event-driven via updateStatus()
    // This function can be used for effects or animations if needed
}

void NeoPixelManager::updateStatus(SystemState state) {
    if (state != last_state_) {
        last_state_ = state;
        uint32_t color = getColorForState(state);
        
        // Set all pixels to the status color
        for (int i = 0; i < NUMPIXELS; i++) {
            pixels_.setPixelColor(i, color);
        }
        pixels_.show();
        
        const char* state_names[] = {"DISCONNECTED", "CONNECTED_SAFE", "READY"};
        DEBUG_PRINTF("NeoPixel status updated: %s", state_names[state]);
    }
}

void NeoPixelManager::setColor(uint8_t r, uint8_t g, uint8_t b) {
    uint32_t color = pixels_.Color(r, g, b);
    for (int i = 0; i < NUMPIXELS; i++) {
        pixels_.setPixelColor(i, color);
    }
    pixels_.show();
}

void NeoPixelManager::setBrightness(uint8_t brightness) {
    pixels_.setBrightness(brightness);
    pixels_.show();
}

void NeoPixelManager::clear() {
    pixels_.clear();
    pixels_.show();
}

uint32_t NeoPixelManager::getColorForState(SystemState state) {
    switch (state) {
        case STATE_DISCONNECTED:
            return pixels_.Color(255, 0, 0);    // Red
        case STATE_CONNECTED_SAFE:
            return pixels_.Color(255, 255, 0);  // Yellow
        case STATE_READY:
            return pixels_.Color(0, 255, 0);    // Green
        default:
            return pixels_.Color(255, 0, 255);  // Magenta (error)
    }
}