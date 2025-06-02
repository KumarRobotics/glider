/*\
*
*
*/

#include <cmath>

namespace glider
{
namespace geodetics
{

double gpsHeading(double lat1, double lon1, double lat2, double lon2)
{
    double lat1_rad = lat1 * M_PI / 180.0;
    double lon1_rad = lon1 * M_PI / 180.0;
    double lat2_rad = lat2 * M_PI / 180.0;
    double lon2_rad = lon2 * M_PI / 180.0;

    double lon_diff = lon2_rad - lon1_rad;

    double y = std::sin(lon_diff) * std::cos(lat2_rad);
    double x = std::cos(lat1_rad) * std::sin(lat2_rad) - std::sin(lat1_rad) * std::cos(lat2_rad) * std::cos(lon_diff);

    double heading_rad = std::atan2(y,x);

    return heading_rad;
}

double headingRadiansToDegrees(double heading, bool use_enu = true)
{
    double heading_deg = heading * (180.0 / M_PI);
    
    if (heading_deg < 0.0)
    {
        heading_deg = heading_deg + 360.0;
    }
    else if (heading_deg > 360.0)
    {
        heading_deg = heading_deg - 360.0;
    }

    return heading_deg;
}

double geodeticToENU(double geodetic_heading)
{
    double enu_heading = std::fmod((M_PI/2 - geodetic_heading + (2*M_PI)), (2*M_PI));
    
    return enu_heading;
}
} // namespace geodetics
} // namespace glider
