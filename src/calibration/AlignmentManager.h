#ifndef ALIGNMENT_MANAGER_H
#define ALIGNMENT_MANAGER_H

// CRITICAL: specific headers must be included BEFORE others to prevent macro collisions
#include <Arduino.h> 

class AlignmentManager {
public:
    static void init();
    static void update();
    
    // GPS / Base Station Methods
    static bool isAcquiring();
    static int getAcquisitionRemaining(); // seconds
    static void startAcquisition();
    
    static bool isBaseSet();
    static double getHomeLat();
    static double getHomeLon();
    
    // Alignment Correction Methods
    static void setTare(double targetLat, double targetLon);
    static void setVisualOffset(float offsetDeg);
    
private:
    static bool initialized;
};

#endif // ALIGNMENT_MANAGER_H
