/*
*
*
*/

#include <gtest/gtest.h>

#include "glider/utils/gps_heading.hpp"

TEST(HeadingTestSuite, GpsHeading)
{
    double nylat = 40.7128;
    double nylon = -74.0060;
    double londonlat = 51.5074;
    double londonlon = -0.1278;

    double heading = glider::geodetics::gpsHeading(nylat, nylon, londonlat, londonlon);

    EXPECT_NEAR(heading, 51.21, 0.01);
}
