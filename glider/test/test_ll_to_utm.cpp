/*
* Jason Hughes
* April 2025
*
* A unit test to get Latitude and Longitude conversion to UTM
*/
#include <gtest/gtest.h>

#include "glider/utils/geodetics.hpp"

const double lat = 39.941423;
const double lon = -75.199414;

const double gt_easting = 482963.5387620803;
const double gt_northing = 4421274.811364001;
const char zone[4] = "18S";

const double tolerance = 0.0001;


TEST(ConversionTestSuite, LatLonToEasting)
{
    double easting, t0;
    char t1[4];

    glider::geodetics::LLtoUTM(lat, lon, t0, easting, t1);
    ASSERT_NEAR(easting, gt_easting, tolerance);
}

TEST(ConversionTestSuite, LatLonToNorthing)
{
    double northing, t0;
    char t1[4];

    glider::geodetics::LLtoUTM(lat, lon, northing, t0, t1);
    ASSERT_NEAR(northing, gt_northing, tolerance);
}

TEST(ConversionTestSuite, UTMToLatitude)
{
    double plat, temp;
    glider::geodetics::UTMtoLL(gt_northing, gt_easting, zone, plat, temp);
    ASSERT_NEAR(lat, plat, tolerance);
}

TEST(ConversionvTestSuite, UTMToLongitude)
{
    double plon, temp;
    glider::geodetics::UTMtoLL(gt_northing, gt_easting, zone, temp, plon);
    ASSERT_NEAR(lon, plon, tolerance);
}
