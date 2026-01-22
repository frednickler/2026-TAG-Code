#ifndef SERIAL_MENU_H
#define SERIAL_MENU_H

#include <Arduino.h>
#include "../config/SystemConfig.h"

/**
 * @brief Non-Blocking Serial Menu System
 * 
 * Character-by-character parser that doesn't block the main loop.
 * User can type commands while sensors continue to update.
 * 
 * Main Commands:
 * - 'm' or 'menu': Show main menu
 * - 'h' or 'help': Show help
 * - 's' or 'status': Show sensor status
 * - 'c': Calibration menu
 * - 'cfg': Configuration menu
 */
class SerialMenu {
public:
    /**
     * @brief Initialize the menu system
     */
    static void init();
    
    /**
     * @brief Update menu (call regularly in main loop)
     * Non-blocking - processes one character at a time
     */
    static void update();
    
    /**
     * @brief Check if menu is currently active
     * @return true if in menu mode
     */
    static bool isActive();
    
    /**
     * @brief Show main menu
     */
    static void showMainMenu();

    /**
     * @brief Check if data logging is active
     * @return true if currently logging data to serial as CSV
     */
    static bool isDataLogging();
    
    /**
     * @brief Start data logging (prints CSV header)
     */
    static void startLogging();
    
    /**
     * @brief Stop data logging (prints summary)
     */
    static void stopLogging();
    
    /**
     * @brief Increment sample counter (call from main loop when logging a line)
     */
    static void incrementLogSample();
    
    /**
     * @brief Show sensor status (public for main.cpp)
     */
    static void showStatus();
    
private:
    static bool active;
    static bool logging;           // Data logger active flag
    static unsigned long logStartTime;  // When logging started (for duration calc)
    static unsigned long logSamples;    // Number of samples logged
    static String inputBuffer;
    static unsigned long lastInputTime;
    
    static void processCommand(const String& cmd);
    static void showHelp();
    static void showCalibrationMenu();
    static void showConfigMenu();
    
    // Configuration Sub-menus
    static void showIMUConfig();
    static void showMagConfig();           // Magnetometer submenu
    static void showMagSimpleConfig();     // Simple presets
    static void showMagAdvancedConfig();   // Advanced manual config
    static void showGPSConfig();
    static void showBaroConfig();
    static void showSystemConfig();
    static void showVQFConfig();
    
    // Alignment Menu
    static void showAlignmentMenu();
    
    // Output Mode Menu
    static void showOutputModeMenu();
    
    // Data Logger Menu
    static void showDataLoggerMenu();
    
    // Radio (ESP-NOW) Menu
    static void showRadioMenu();
    static void showRadioMACMenu();
    static void showRadioAdvancedMenu();
};

#endif // SERIAL_MENU_H
