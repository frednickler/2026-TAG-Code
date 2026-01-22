#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include <Arduino.h>

// =============================================================================
// SERVO CONTROLLER
// =============================================================================
// NOTE: This is a placeholder module. The actual servo GPIO and PWM control
// will be implemented later when the hardware is installed.
// 
// TODO (FUTURE): 
//   - Define SERVO_GPIO_PIN in SystemConfig.h
//   - Add servo configuration to RuntimeConfig (min/max angle, speed, etc.)
//   - Implement PWM control using ESP32 LEDC peripheral
//   - Add calibration for servo center position
// =============================================================================

class ServoController {
public:
    // Lifecycle
    static void init();
    static void update();
    
    // Control
    static void setTargetAngle(float degrees);    // Set desired position (-180 to +180)
    static void setEnabled(bool enabled);
    
    // Status
    static float getTargetAngle();
    static float getCurrentAngle();
    static bool isEnabled();
    static bool isHardwareConnected();  // Always false until hardware installed
    
    // Tracking calculation (uses RadioManager and GPSManager data)
    static void calculateTrackingAngle();
    static float getDistanceToTag();
    static float getBearingToTag();
    
private:
    static bool enabled;
    static float targetAngle;
    static float currentAngle;
    static float distanceToTag;
    static float bearingToTag;
    
    // Smoothing
    static float smoothAngle(float current, float target, float maxStep);
};

#endif // SERVO_CONTROLLER_H
