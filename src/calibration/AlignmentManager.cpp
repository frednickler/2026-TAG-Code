#include "AlignmentManager.h"
#include "../config/SystemConfig.h"
#include "../config/SystemSettings.h"
#include "../system/SerialMenu.h"

// Static members
bool AlignmentManager::initialized = false;
static bool _acquiring = false;
static bool _baseSet = false;
static unsigned long _acquisitionStartTime = 0;
static double _homeLat = 0.0;
static double _homeLon = 0.0;

void AlignmentManager::init() {
    if (initialized) return;
    DEBUG_INFO("Alignment Manager Initialized");
    initialized = true;
}

void AlignmentManager::update() {
    if (!initialized) return;
    
    // Simple mock logic for acquisition timeout
    if (_acquiring) {
        if (millis() - _acquisitionStartTime > 60000) { // 60s timeout
            _acquiring = false;
            _baseSet = true;
            // Mock home position
            _homeLat = -34.0; 
            _homeLon = 151.0;
            DEBUG_INFO("Alignment Acquisition Complete (Mock)");
        }
    }
}

bool AlignmentManager::isAcquiring() { return _acquiring; }
int AlignmentManager::getAcquisitionRemaining() {
    if (!_acquiring) return 0;
    unsigned long elapsed = millis() - _acquisitionStartTime;
    return (60000 - elapsed) / 1000;
}

void AlignmentManager::startAcquisition() {
    _acquiring = true;
    _baseSet = false;
    _acquisitionStartTime = millis();
    DEBUG_INFO("Started Alignment Acquisition");
}

bool AlignmentManager::isBaseSet() { return _baseSet; }
double AlignmentManager::getHomeLat() { return _homeLat; }
double AlignmentManager::getHomeLon() { return _homeLon; }

void AlignmentManager::setTare(double lat, double lon) {
    DEBUG_INFO("Tare set to: %.6f, %.6f", lat, lon);
    // TODO: Implement actual heading calculation
}

void AlignmentManager::setVisualOffset(float offset) {
    RuntimeConfig& cfg = SystemSettings::getConfig();
    cfg.savedHeadingOffset = offset;
    SystemSettings::save();
    DEBUG_INFO("Visual Offset saved: %.1f", offset);
}
