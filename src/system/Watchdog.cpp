#include "Watchdog.h"

bool Watchdog::initialized = false;

bool Watchdog::init() {
    DEBUG_INFO("Initializing Watchdog Timer (timeout: %d sec)", WATCHDOG_TIMEOUT_SEC);
    
    // Initialize Task Watchdog Timer (legacy API for Arduino ESP32)
    esp_err_t err = esp_task_wdt_init(WATCHDOG_TIMEOUT_SEC, true);  // timeout_s, panic_on_timeout
    if (err != ESP_OK) {
        DEBUG_ERROR("Failed to initialize WDT: %d", err);
        return false;
    }
    
    // Subscribe current task to the watchdog
    err = esp_task_wdt_add(NULL);  // NULL = current task
    if (err != ESP_OK) {
        DEBUG_ERROR("Failed to add task to WDT: %d", err);
        return false;
    }
    
    initialized = true;
    DEBUG_INFO("Watchdog Timer initialized successfully");
    return true;
}

void Watchdog::feed() {
    if (!initialized) {
        DEBUG_WARN("Attempted to feed uninitialized watchdog");
        return;
    }
    
    // Reset the watchdog timer
    esp_task_wdt_reset();
}

bool Watchdog::isInitialized() {
    return initialized;
}

void Watchdog::disable() {
    if (!initialized) return;
    
    DEBUG_WARN("DISABLING WATCHDOG - USE ONLY FOR DEBUGGING!");
    esp_task_wdt_delete(NULL);
    esp_task_wdt_deinit();
    initialized = false;
}

bool Watchdog::setExtendedTimeout(uint8_t seconds) {
    if (!initialized) return false;
    
    // Limit to 60 seconds max
    if (seconds > 60) seconds = 60;
    if (seconds < 1) seconds = 1;
    
    DEBUG_INFO("Extending watchdog timeout to %d seconds", seconds);
    
    // Remove task from current WDT, deinit, reinit with new timeout
    esp_task_wdt_delete(NULL);
    esp_task_wdt_deinit();
    
    esp_err_t err = esp_task_wdt_init(seconds, true);
    if (err != ESP_OK) {
        DEBUG_ERROR("Failed to reinit WDT with extended timeout: %d", err);
        return false;
    }
    
    err = esp_task_wdt_add(NULL);
    if (err != ESP_OK) {
        DEBUG_ERROR("Failed to re-add task to WDT: %d", err);
        return false;
    }
    
    return true;
}

void Watchdog::restoreDefaultTimeout() {
    if (!initialized) return;
    
    DEBUG_INFO("Restoring watchdog timeout to %d seconds", WATCHDOG_TIMEOUT_SEC);
    
    esp_task_wdt_delete(NULL);
    esp_task_wdt_deinit();
    
    esp_err_t err = esp_task_wdt_init(WATCHDOG_TIMEOUT_SEC, true);
    if (err == ESP_OK) {
        esp_task_wdt_add(NULL);
    }
}
