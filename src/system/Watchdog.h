#ifndef WATCHDOG_H
#define WATCHDOG_H

#include <Arduino.h>
#include "esp_task_wdt.h"
#include "../config/SystemConfig.h"

/**
 * @brief Watchdog Timer Manager
 * 
 * Wraps ESP32 Task Watchdog Timer (TWDT) for safety monitoring.
 * The main loop MUST call feed() regularly or the system will reset.
 */
class Watchdog {
public:
    /**
     * @brief Initialize the watchdog timer
     * @return true if successful, false otherwise
     */
    static bool init();
    
    /**
     * @brief Feed the watchdog (reset timer)
     * Call this regularly in the main loop to prevent reset
     */
    static void feed();
    
    /**
     * @brief Check if watchdog is initialized
     * @return true if initialized
     */
    static bool isInitialized();
    
    /**
     * @brief Disable the watchdog (USE WITH CAUTION)
     * Only for emergency debugging
     */
    static void disable();
    
    /**
     * @brief Temporarily extend timeout for long operations
     * @param seconds New timeout in seconds (max 60)
     * @return true if successful
     */
    static bool setExtendedTimeout(uint8_t seconds);
    
    /**
     * @brief Restore the default timeout
     */
    static void restoreDefaultTimeout();

private:
    static bool initialized;
};

#endif // WATCHDOG_H
