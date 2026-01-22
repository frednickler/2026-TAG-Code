#include "ServoController.h"
#include "../config/SystemConfig.h"
#include "../config/SystemSettings.h"
#include "../radio/RadioManager.h"
#include "../sensors/GPSManager.h"
#include "../sensors/IMUManager.h"
#include "../utils/GeoMath.h"
#include "../calibration/AlignmentManager.h"

// =============================================================================
// STATIC MEMBER INITIALIZATION
// =============================================================================
bool ServoController::enabled = false;
float ServoController::targetAngle = 0.0f;
float ServoController::currentAngle = 0.0f;
float ServoController::distanceToTag = 0.0f;
float ServoController::bearingToTag = 0.0f;

// =============================================================================
// LIFECYCLE
// =============================================================================

void ServoController::init() {
    // NOTE: Hardware not connected yet - this is a placeholder
    // TODO: When hardware is installed:
    //   1. Configure GPIO pin for servo PWM
    //   2. Set up LEDC peripheral
    //   3. Calibrate center position
    
    enabled = false;  // Disabled until hardware connected
    targetAngle = 0.0f;
    currentAngle = 0.0f;
    
    DEBUG_INFO("ServoController initialized (PLACEHOLDER - no hardware)");
}

void ServoController::update() {
    // Always calculate tracking angle for display purposes
    calculateTrackingAngle();
    
    if (!enabled) {
        return;
    }
    
    // TODO: When hardware is installed:
    //   1. Smooth current angle towards target
    //   2. Convert angle to PWM duty cycle
    //   3. Write to servo
    
    // Placeholder: Instant movement (for software simulation)
    currentAngle = smoothAngle(currentAngle, targetAngle, 5.0f);  // 5 deg/update max
}

// =============================================================================
// CONTROL
// =============================================================================

void ServoController::setTargetAngle(float degrees) {
    // Clamp to valid range
    if (degrees < -180.0f) degrees = -180.0f;
    if (degrees > 180.0f) degrees = 180.0f;
    targetAngle = degrees;
}

void ServoController::setEnabled(bool en) {
    enabled = en;
    if (en) {
        DEBUG_INFO("Servo enabled");
    } else {
        DEBUG_INFO("Servo disabled");
    }
}

// =============================================================================
// STATUS
// =============================================================================

float ServoController::getTargetAngle() {
    return targetAngle;
}

float ServoController::getCurrentAngle() {
    return currentAngle;
}

bool ServoController::isEnabled() {
    return enabled;
}

bool ServoController::isHardwareConnected() {
    // TODO: Return true when hardware is installed and detected
    return false;
}

float ServoController::getDistanceToTag() {
    return distanceToTag;
}

float ServoController::getBearingToTag() {
    return bearingToTag;
}

// =============================================================================
// TRACKING CALCULATION
// =============================================================================

void ServoController::calculateTrackingAngle() {
    // Only calculate if we have valid data from both BASE GPS and TAG radio
    if (!AlignmentManager::isBaseSet()) {
        return;
    }
    
    if (!RadioManager::isConnected()) {
        return;
    }
    
    // Get BASE position (home position from AlignmentManager)
    double baseLat = AlignmentManager::getHomeLat();
    double baseLon = AlignmentManager::getHomeLon();
    
    // Get TAG position from radio
    double tagLat = RadioManager::getTagLatitude();
    double tagLon = RadioManager::getTagLongitude();
    
    // Skip if TAG position is invalid (0,0)
    if (tagLat == 0.0 && tagLon == 0.0) {
        return;
    }
    
    // Calculate distance and bearing from BASE to TAG
    distanceToTag = GeoMath::distance(baseLat, baseLon, tagLat, tagLon);
    bearingToTag = GeoMath::bearing(baseLat, baseLon, tagLat, tagLon);
    
    // Get current BASE heading from IMU
    float baseHeading = IMUManager::getCompassHeading();
    
    // Calculate relative angle: where TAG is relative to BASE's heading
    // Positive = TAG is to the right, Negative = TAG is to the left
    float relativeAngle = GeoMath::deltaAngle(baseHeading, bearingToTag);
    
    // Apply visual offset (TARE)
    RuntimeConfig& cfg = SystemSettings::getConfig();
    float finalAngle = relativeAngle + cfg.savedHeadingOffset;
    
    // Normalize to -180 to +180
    if (finalAngle > 180.0f) finalAngle -= 360.0f;
    if (finalAngle < -180.0f) finalAngle += 360.0f;
    
    // Set servo target
    setTargetAngle(finalAngle);
}

// =============================================================================
// INTERNAL HELPERS
// =============================================================================

float ServoController::smoothAngle(float current, float target, float maxStep) {
    float diff = target - current;
    
    // Handle wrap-around
    if (diff > 180.0f) diff -= 360.0f;
    if (diff < -180.0f) diff += 360.0f;
    
    // Limit step size
    if (diff > maxStep) diff = maxStep;
    if (diff < -maxStep) diff = -maxStep;
    
    float result = current + diff;
    
    // Normalize
    if (result > 180.0f) result -= 360.0f;
    if (result < -180.0f) result += 360.0f;
    
    return result;
}
