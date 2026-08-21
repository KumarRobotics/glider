/*
* Test the factor manager
*
*/

#include <gtest/gtest.h>

#include "glider/core/factor_manager.hpp"

TEST(FactorManagerTestSuite, ImuInitialization)
{
    // initialized glider factor manager and params
    Glider::Parameters params = Glider::Parameters::Load("../config/glider-params.yaml");
    Glider::FactorManager manager(params);

    // provide measurements for initialization
    for (int i = 0; i < params.bias_num_measurements + 10; ++i)
    {
        Eigen::Vector3d accel(0.0, 0.0, params.gravity);
        Eigen::Vector3d gyro(0.0, 0.0, 0.0);
        Eigen::Vector4d orient(1.0, 0.0, 0.0, 0.0);
        int64_t timestamp = i;
        manager.addImuFactor(timestamp, accel, gyro, orient);
    }

    // assert its initialized
    ASSERT_TRUE(manager.isImuInitialized());

    // get bias and compute mean
    Eigen::MatrixXd bias_est = manager.getBiasEstimate();

    Eigen::Vector3d accel_bias(bias_est.colwise().mean().head(3));
    Eigen::Vector3d gyro_bias(bias_est.colwise().mean().tail(3));

    Eigen::Vector3d accel_gt(0.0, 0.0, 0.0);
    Eigen::Vector3d gyro_gt(0.0, 0.0, 0.0);

    // check bias calculation is correct
    ASSERT_EQ(accel_bias, accel_gt);
    ASSERT_EQ(gyro_bias, gyro_gt);
}

TEST(FactorManagerTestSuite, ImuInitializationWithRotatedMount)
{
    Glider::Parameters params = Glider::Parameters::Load("../config/glider-params.yaml");
    Glider::FactorManager manager(params);

    // A +90 degree body-to-ENU rotation about X maps body +Y to ENU +Z.
    const double s = std::sqrt(0.5);
    Eigen::Vector4d orient(s, s, 0.0, 0.0);
    for (int i = 0; i < params.bias_num_measurements + 1; ++i)
    {
        Eigen::Vector3d accel(0.0, params.gravity, 0.0);
        Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
        manager.addImuFactor(i, accel, gyro, orient);
    }

    ASSERT_TRUE(manager.isImuInitialized());
    const Eigen::Vector3d accel_bias =
        manager.getBiasEstimate().colwise().mean().head(3);
    EXPECT_LT(accel_bias.norm(), 1e-9);
}

TEST(FactorManagerTestSuite, PimParameters)
{
    // initialized glider factor manager and params
    Glider::Parameters params = Glider::Parameters::Load("../config/glider-params.yaml");
    Glider::FactorManager manager(params);

    // provide measurements for initialization
    for (int i = 0; i < params.bias_num_measurements + 1; ++i)
    {
        Eigen::Vector3d accel(0.0, 0.0, params.gravity);
        Eigen::Vector3d gyro(0.0, 0.0, 0.0);
        Eigen::Vector4d orient(1.0, 0.0, 0.0, 0.0);
        int64_t timestamp = i;
        manager.addImuFactor(timestamp, accel, gyro, orient);
    }
    // make sure the imu is initialized,
    // otherwise nothing will get added to the pim
    ASSERT_TRUE(manager.isImuInitialized());

    // add 10 basic measurements to the pim
    for (int i = 0; i < 10; ++i)
    {
        Eigen::Vector3d accel(0.0, 0.0, params.gravity);
        Eigen::Vector3d gyro(0.0, 0.0, 0.0);
        Eigen::Vector4d orient(1.0, 0.0, 0.0, 0.0);
        int64_t timestamp = i;
        manager.addImuFactor(timestamp, accel, gyro, orient);
    }

    // get the pim from the factor manager
    gtsam::PreintegratedCombinedMeasurements pim = manager.getPim();

    // test that the velocity is 0 nad pose are zero
    Eigen::Vector3d vel_gt = Eigen::Vector3d::Zero();
    Eigen::Vector3d pos_gt = Eigen::Vector3d::Zero();

    ASSERT_EQ(pim.deltaPij(), pos_gt);
    ASSERT_EQ(pim.deltaVij(), vel_gt);
}

TEST(FactorManagerTestSuite, KeyIndex)
{
    // set default lat lon
    double lat = 39.941279;
    double lon = -75.199197;

    // initialized glider factor manager and params
    Glider::Parameters params = Glider::Parameters::Load("../config/glider-params.yaml");
    Glider::FactorManager manager(params);

    ASSERT_EQ(manager.getKeyIndex(), 0);
    // provide measurements for initialization
    for (int i = 0; i < params.bias_num_measurements + 1; ++i)
    {
        Eigen::Vector3d accel(0.0, 0.0, params.gravity);
        Eigen::Vector3d gyro(0.0, 0.0, 0.0);
        Eigen::Vector4d orient(1.0, 0.0, 0.0, 0.0);
        int64_t timestamp = i;
        manager.addImuFactor(timestamp, accel, gyro, orient);
    }
    ASSERT_EQ(manager.getKeyIndex(), 0);

    Eigen::Vector3d meas(lat, lon, 0.0);
    manager.addGpsFactor(1, meas);

    ASSERT_EQ(manager.getKeyIndex(), 1);
}

TEST(FactorManagerTestSuite, GPSInitialization)
{
    // set default lat lon
    double lat = 39.941279;
    double lon = -75.199197;

    // initialized glider factor manager and params
    Glider::Parameters params = Glider::Parameters::Load("../config/glider-params.yaml");
    Glider::FactorManager manager(params);

    // provide imu measurements for initialization
    for (int i = 0; i < params.bias_num_measurements + 1; ++i)
    {
        Eigen::Vector3d accel(0.0, 0.0, params.gravity);
        Eigen::Vector3d gyro(0.0, 0.0, 0.0);
        Eigen::Vector4d orient(1.0, 0.0, 0.0, 0.0);
        int64_t timestamp = i;
        manager.addImuFactor(timestamp, accel, gyro, orient);
    }

    ASSERT_FALSE(manager.isGpsInitialized());

    Eigen::Vector3d meas(lat, lon, 0);
    manager.addGpsFactor(1, meas);

    ASSERT_TRUE(manager.isGpsInitialized());
}

TEST(FactorManagerTestSuite, SystemInitialization)
{
    // set default lat lon
    double lat = 39.941279;
    double lon = -75.199197;

    // initialized glider factor manager and params
    Glider::Parameters params = Glider::Parameters::Load("../config/glider-params.yaml");
    Glider::FactorManager manager(params);

    // assert system is NOT initialized
    ASSERT_FALSE(manager.isSystemInitialized());

    for (uint64_t i = 0; i < params.initial_num_measurements + 1; ++i)
    {
        // provide imu measurements for initialization
        for (int j = 0; j < params.bias_num_measurements + 1; ++j)
        {
            Eigen::Vector3d accel(0.0, 0.0, params.gravity);
            Eigen::Vector3d gyro(0.0, 0.0, 0.0);
            Eigen::Vector4d orient(1.0, 0.0, 0.0, 0.0);
            int64_t timestamp = (i+1) * (j+1);
            manager.addImuFactor(timestamp, accel, gyro, orient);
        }
        Eigen::Vector3d meas(lat, lon, 0.0);
        manager.addGpsFactor(i+1, meas);
        int64_t timestamp = 1;
        Glider::OdometryWithCovariance state = manager.runner(timestamp);
    }

    // After adding the specified amount of gps measurements
    // the system should initialize
    ASSERT_TRUE(manager.isSystemInitialized());
}
