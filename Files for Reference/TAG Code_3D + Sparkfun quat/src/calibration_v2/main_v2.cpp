#include <Arduino.h>
#include "CalibrationEngine.h"

void setup() {
    CalibrationEngine::begin();
}

void loop() {
    CalibrationEngine::loop();
}
