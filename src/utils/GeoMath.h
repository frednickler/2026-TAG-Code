#ifndef GEO_MATH_H
#define GEO_MATH_H

#include <Arduino.h>

class GeoMath {
public:
    // Constants
    static constexpr double EARTH_RADIUS_M = 6371000.0;
    static constexpr double DEGREES_TO_RADIANS = 0.017453292519943295769236907684886;
    static constexpr double RADIANS_TO_DEGREES = 57.295779513082320876798154814105;

    /**
     * @brief Calculate distance between two points in meters using Haversine formula
     */
    static double distance(double lat1, double lon1, double lat2, double lon2);

    /**
     * @brief Calculate initial bearing (forward azimuth) from point 1 to point 2
     * @return Bearing in degrees (0-360)
     */
    static double bearing(double lat1, double lon1, double lat2, double lon2);

    /**
     * @brief Normalize angle to 0-360 degrees
     */
    static float normalizeAngle(float angle);

    /**
     * @brief Calculate shortest turn angle (-180 to +180) from current to target
     */
    static float deltaAngle(float current, float target);
};

#endif // GEO_MATH_H
