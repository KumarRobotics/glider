/*!
* Jason Hughes
* October 2025
* 
* @brief unit test for the
* OdometryWithCovariance object
*/

#include <gtest/gtest.h>
#include <Eigen/Eigenvalues>

#include "glider/core/odometry_with_covariance.hpp"
#include "glider/core/factor_manager.hpp"

static const double LATITUDE = 39.941259;
static const double LONGITUDE = -75.199202;
static const char ZONE[4] = "18S";
static const double TOL = 0.0001;

static const double TX = 482981.6098624719;
static const double TY = 4421256.568684888;
static const double TZ = 10.0;

static const double VX = 0.0;
static const double VY = 0.0;
static const double VZ = 0.0;

static const double QW = 1.0;
static const double QX = 0.0;
static const double QY = 0.0;
static const double QZ = 0.0;

static const double AX = 0.0;
static const double AY = 0.0;
static const double AZ = 9.81;

static const double GX = 0.0;
static const double GY = 0.0;
static const double GZ = 0.0;

TEST(OdometryWithCovarianceTestSuite, TestInitialization)
{
    // initialized glider factor manager and params
    Glider::Parameters params = Glider::Parameters::Load("../config/glider-params.yaml");
    Glider::FactorManager manager(params);

    Glider::OdometryWithCovariance odom;
    
    ASSERT_FALSE(odom.isInitialized());

    for (uint64_t i = 0; i < params.initial_num_measurements + 1; ++i)
    {
        // provide imu measurements for initialization
        for (int j = 0; j < params.bias_num_measurements + 1; ++j)
        {
            Eigen::Vector3d accel(AX, AY, AZ);
            Eigen::Vector3d gyro(GX, GY, GZ);
            Eigen::Vector4d orient(QW, QX, QY, QZ);
            int64_t timestamp = (i+1) * (j+1);
            manager.addImuFactor(timestamp, accel, gyro, orient);
        }
        Eigen::Vector3d meas(LATITUDE, LONGITUDE, TZ);
        manager.addGpsFactor(i+1, meas);
        odom = manager.runner(1);
    }
    
    ASSERT_TRUE(odom.isInitialized());
}

/*! @note these tests are note exact but should give us an idea 
 *  that things are getting initialized correctly at least*/
TEST(OdometryWithCovarainceTestSuite, TestCovariances)
{
    // initialized glider factor manager and params
    Glider::Parameters params = Glider::Parameters::Load("../config/glider-params.yaml");
    Glider::FactorManager manager(params);

    Glider::OdometryWithCovariance odom;
    int64_t timestamp = 0;

    for (uint64_t i = 0; i < params.initial_num_measurements + 1; ++i)
    {
        // provide imu measurements for initialization
        for (int j = 0; j < params.bias_num_measurements + 1; ++j)
        {
            Eigen::Vector3d accel(AX, AY, AZ);
            Eigen::Vector3d gyro(GX, GY, GZ);
            Eigen::Vector4d orient(QW, QX, QY, QZ);
            timestamp += 10000000;
            manager.addImuFactor(timestamp, accel, gyro, orient);
        }
        Eigen::Vector3d meas(LATITUDE, LONGITUDE, TZ);
        manager.addGpsFactor(timestamp, meas);
        odom = manager.runner(timestamp);
    }

    const auto expect_valid_covariance = [](const Eigen::MatrixXd& covariance) {
        EXPECT_TRUE(covariance.isApprox(covariance.transpose(), 1e-9));
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(covariance);
        ASSERT_EQ(solver.info(), Eigen::Success);
        EXPECT_GE(solver.eigenvalues().minCoeff(), -1e-9);
        EXPECT_GT(covariance.diagonal().sum(), 0.0);
    };

    expect_valid_covariance(odom.getPoseCovariance());
    expect_valid_covariance(odom.getPositionCovariance());
    expect_valid_covariance(odom.getVelocityCovariance());
}

TEST(OdometryWithCovarianceTestSuite, TestBiases)
{
    // initialized glider factor manager and params
    Glider::Parameters params = Glider::Parameters::Load("../config/glider-params.yaml");
    Glider::FactorManager manager(params);

    Glider::OdometryWithCovariance odom;
    
    for (uint64_t i = 0; i < params.initial_num_measurements + 1; ++i)
    {
        // provide imu measurements for initialization
        for (int j = 0; j < params.bias_num_measurements + 1; ++j)
        {
            Eigen::Vector3d accel(AX, AY, AZ);
            Eigen::Vector3d gyro(GX, GY, GZ);
            Eigen::Vector4d orient(QW, QX, QY, QZ);
            int64_t timestamp = (i+1) * (j+1);
            manager.addImuFactor(timestamp, accel, gyro, orient);
        }
        Eigen::Vector3d meas(LATITUDE, LONGITUDE, TZ);
        manager.addGpsFactor(i+1, meas);
        odom = manager.runner(1);
    }
    
    Eigen::Vector3d accel_bias_gt(0.0, 0.0, 0.0);
    Eigen::Vector3d gyro_bias_gt(0.0, 0.0, 0.0);

    const double EPSILON = 1e-10;

    // test get bias
    gtsam::imuBias::ConstantBias gtbias = odom.getBias<gtsam::imuBias::ConstantBias>();
    for (const double& v : gtbias.accelerometer())
    {
        ASSERT_NEAR(v, 0.0, EPSILON);
    }
    for (const double& v : gtbias.gyroscope())
    {
        ASSERT_NEAR(v, 0.0, EPSILON);
    }
    std::pair<Eigen::Vector3d, Eigen::Vector3d> eigpair = odom.getBias<std::pair<Eigen::Vector3d, Eigen::Vector3d>>();
    for (const double& v : eigpair.first)
    {
        ASSERT_NEAR(v, 0.0, EPSILON);
    }
    for (const double& v : eigpair.second)
    {
        ASSERT_NEAR(v, 0.0, EPSILON);
    }

    for (const double& v : odom.getAccelerometerBias<gtsam::Vector3>())
    {
        ASSERT_NEAR(v, 0.0, EPSILON);
    }
    for (const double& v : odom.getAccelerometerBias<Eigen::Vector3d>())
    {
        ASSERT_NEAR(v, 0.0, EPSILON);
    }

    for (const double& v : odom.getGyroscopeBias<gtsam::Vector3>())
    {
        ASSERT_NEAR(v, 0.0, EPSILON);
    }
    for (const double& v: odom.getGyroscopeBias<Eigen::Vector3d>())
    {
        ASSERT_NEAR(v, 0.0, EPSILON);
    }
}

TEST(OdometryWithCovarianceTestSuite, TestKeyIndex)
{
    // initialized glider factor manager and params
    Glider::Parameters params = Glider::Parameters::Load("../config/glider-params.yaml");
    Glider::FactorManager manager(params);

    Glider::OdometryWithCovariance odom;
    int64_t timestamp = 0;

    for (uint64_t i = 0; i < params.initial_num_measurements + 1; ++i)
    {
        // provide imu measurements for initialization
        for (int j = 0; j < params.bias_num_measurements + 1; ++j)
        {
            Eigen::Vector3d accel(AX, AY, AZ);
            Eigen::Vector3d gyro(GX, GY, GZ);
            Eigen::Vector4d orient(QW, QX, QY, QZ);
            timestamp += 10000000;
            manager.addImuFactor(timestamp, accel, gyro, orient);
        }
        Eigen::Vector3d meas(LATITUDE, LONGITUDE, TZ);
        manager.addGpsFactor(timestamp, meas);
        odom = manager.runner(timestamp);
    }

    ASSERT_EQ(odom.getKeyIndex<gtsam::Key>(), params.initial_num_measurements);
    ASSERT_EQ(odom.getKeyIndex<int>(), params.initial_num_measurements);

    std::string pose_key = "x" + std::to_string(params.initial_num_measurements);
    ASSERT_EQ(odom.getKeyIndex("X"), pose_key);

    std::string vel_key = "v" + std::to_string(params.initial_num_measurements);
    ASSERT_EQ(odom.getKeyIndex("V"), vel_key);

    std::string bias_key = "b" + std::to_string(params.initial_num_measurements);
    ASSERT_EQ(odom.getKeyIndex("B"), bias_key);
}

TEST(OdometryWithCovarianceTestSuite, TestMovementStatus)
{ 
    // initialized glider factor manager and params
    Glider::Parameters params = Glider::Parameters::Load("../config/glider-params.yaml");
    Glider::FactorManager manager(params);

    Glider::OdometryWithCovariance odom;
    
    for (uint64_t i = 0; i < params.initial_num_measurements + 1; ++i)
    {
        // provide imu measurements for initialization
        for (int j = 0; j < params.bias_num_measurements + 1; ++j)
        {
            Eigen::Vector3d accel(AX, AY, AZ);
            Eigen::Vector3d gyro(GX, GY, GZ);
            Eigen::Vector4d orient(QW, QX, QY, QZ);
            int64_t timestamp = (i+1) * (j+1);
            manager.addImuFactor(timestamp, accel, gyro, orient);
        }
        Eigen::Vector3d meas(LATITUDE, LONGITUDE, TZ);
        manager.addGpsFactor(i+1, meas);
        odom = manager.runner(1);
    }

    ASSERT_FALSE(odom.isMoving());
    Eigen::Vector3d test(0.0,0.0,0.0);
    ASSERT_FALSE(odom.isMovingFasterThan(1.0));
}
