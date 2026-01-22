#include "GeoMath.h"
#include <math.h>

double GeoMath::distance(double lat1, double lon1, double lat2, double lon2) {
    double dLat = (lat2 - lat1) * DEGREES_TO_RADIANS;
    double dLon = (lon2 - lon1) * DEGREES_TO_RADIANS;

    double a = sin(dLat / 2.0) * sin(dLat / 2.0) +
               cos(lat1 * DEGREES_TO_RADIANS) * cos(lat2 * DEGREES_TO_RADIANS) *
               sin(dLon / 2.0) * sin(dLon / 2.0);
    
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    
    return EARTH_RADIUS_M * c;
}

double GeoMath::bearing(double lat1, double lon1, double lat2, double lon2) {
    double phi1 = lat1 * DEGREES_TO_RADIANS;
    double phi2 = lat2 * DEGREES_TO_RADIANS;
    double lam1 = lon1 * DEGREES_TO_RADIANS;
    double lam2 = lon2 * DEGREES_TO_RADIANS;

    double y = sin(lam2 - lam1) * cos(phi2);
    double x = cos(phi1) * sin(phi2) -
               sin(phi1) * cos(phi2) * cos(lam2 - lam1);
    
    double theta = atan2(y, x);
    
    return normalizeAngle(theta * RADIANS_TO_DEGREES);
}

float GeoMath::normalizeAngle(float angle) {
    angle = fmod(angle, 360.0f);
    if (angle < 0) angle += 360.0f;
    return angle;
}

float GeoMath::deltaAngle(float current, float target) {
    float diff = target - current;
    diff = normalizeAngle(diff);
    if (diff > 180.0f) diff -= 360.0f;
    return diff;
}
