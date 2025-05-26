/*
* Unit test for parameters
*
*/
#include <iostream>
#include <gtest/gtest.h>

#include "glider/utils/params.hpp"

Params params;

TEST(ParamsTestSuite, DoubleGravity)
{
    /* Test that you load the parameters file correctly
    *  by makeing sure gravity is 9.81
    */
    std::map<std::string, double> p;
    p = params.load<double>("/home/jason/ws/src/glider/test/params.yaml");
    std::cout << p["gravity"] << std::endl;
    ASSERT_EQ(p["gravity"], 9.81);
}
