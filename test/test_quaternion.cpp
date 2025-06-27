/*
*
*   Test quaternion operations
*/ 

#include <gtest/gtest.h>

#include "glider/utils/quaternion.hpp"

using namespace glider;

TEST(QuaternionTestSuite, QuaternionMultiplication)
{
    Quaternion q1(0.0, 0.0, 0.0, 1.0);
    Quaternion q2(0.0, 0.7071, 0.0, 0.7071); 
    
    Quaternion gt(0.0, 0.7071, 0.0, 0.7071);
    Quaternion ans = q1 * q2;

    EXPECT_EQ(ans.x, gt.x);
    EXPECT_EQ(ans.y, gt.y);
    EXPECT_EQ(ans.z, gt.z);
    EXPECT_EQ(ans.w, gt.w);
}

TEST(QuaternionTestSuite, QuaternionSubtraction)
{
    Quaternion q1(0.0, 0.0, 0.0, 1.0);
    Quaternion q2(0.0, 0.7071, 0.0, 0.7071); 
 
    Quaternion gt(0.0, 0.7071, 0.0, 0.7071);
    Quaternion ans = q1 - q2;
    
    EXPECT_EQ(ans.x, gt.x);
    EXPECT_EQ(ans.y, gt.y);
    EXPECT_EQ(ans.z, gt.z);
    EXPECT_EQ(ans.w, gt.w);
}
