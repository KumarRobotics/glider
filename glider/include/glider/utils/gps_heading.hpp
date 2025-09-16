/*\
*
*
*/
#pragma once

#include <cmath>

namespace Glider
{
namespace geodetics
{
    double gpsHeading(double lat1, double lon1, double lat2, double lon2);
    double headingRadiansToDegrees(double heading);
    double geodeticToENU(double geodetic_heading);
} // namespace geodetics
} // namespace glider
