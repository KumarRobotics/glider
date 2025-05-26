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

    double heading_rad = std::atan2(y, x);
    double heading_deg = heading_rad * (180.0 / M_PI);
    double heading_normalized = std::fmod(heading_deg + 360.0, 360);

    return heading_normalized;
}

} // namespace geodetics
} // namespace glider
