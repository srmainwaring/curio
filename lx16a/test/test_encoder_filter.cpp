#include "config.h"
#include "lx16a/lx16a_encoder_filter_python.h"
#include "lx16a/lx16a_pybind11_embed.h"
#include <gtest/gtest.h>
#include <pybind11/pybind11.h>
#include <pybind11/embed.h>
#include <ros/ros.h>

namespace py = pybind11;

// Test fixture
class EncoderFilterPythonTestSuite : public ::testing::Test
{
protected:
    ~EncoderFilterPythonTestSuite() override
    {
    }

    EncoderFilterPythonTestSuite()
    {
        // Load the test data.

    }

    void SetUp() override
    {        
        switch (lx16a::getRosPythonVersion())
        {
          case 2:
          {
            classifier_filename_ = lx16a::getDataPath()
              .append("/lx16a_tree_classifier.joblib");
            regressor_filename_ = lx16a::getDataPath()
              .append("/lx16a_tree_regressor.joblib");
            break;
          }
          case 3:
          {
            classifier_filename_ = lx16a::getDataPath()
              .append("/lx16a_tree_classifier_python3.joblib");
            regressor_filename_ = lx16a::getDataPath()
              .append("/lx16a_tree_regressor_python3.joblib");
            break;
          }
          default:
            break;
        }
    }

    void TearDown() override
    {        
    }

    const uint8_t servo_id_ = 11;
    const int16_t window_ = 10;
    const std::string test_data_filename_ = lx16a::getDataPath()
        .append("/lx16a_raw_data_05.csv");

    std::string classifier_filename_;
    std::string regressor_filename_;
};


// Declare tests
TEST_F(EncoderFilterPythonTestSuite, testConstructor)
{
    std::unique_ptr<lx16a::LX16AEncoderFilter> filter(
        new lx16a::LX16AEncoderFilterPython(
            classifier_filename_,
            regressor_filename_,
            window_));
    filter->init();
    filter->add(servo_id_);

    EXPECT_EQ(filter->getRevolutions(servo_id_), 0) << "revolutions != 0";
    EXPECT_EQ(filter->getCount(servo_id_), 0) << "count != 0";
    EXPECT_EQ(filter->getDuty(servo_id_), 0) << "duty != 0";
    EXPECT_EQ(filter->getInvert(servo_id_), 1) << "invert != 1";

    int16_t position = 0;
    bool is_valid = false;
    filter->getServoPosition(servo_id_, position, is_valid);
    EXPECT_EQ(position, 0) << "position != 0";
    EXPECT_TRUE(is_valid) << "is_valid != true";
}

TEST_F(EncoderFilterPythonTestSuite, testUpdate)
{
    std::unique_ptr<lx16a::LX16AEncoderFilter> filter(
        new lx16a::LX16AEncoderFilterPython(
            classifier_filename_,
            regressor_filename_,
            window_));
    filter->init();
    filter->add(servo_id_);

    int16_t duty = 500;
    int16_t position = 0;
    bool is_valid = false;
    ros::Time start = ros::Time();

    filter->update(servo_id_, start + ros::Duration(0.0), duty, 0);
    filter->getServoPosition(servo_id_, position, is_valid);
    EXPECT_EQ(position, 0) << "position != 0";
    EXPECT_TRUE(is_valid) << "is_valid != true";
    EXPECT_EQ(filter->getRevolutions(servo_id_), 0) << "revolutions != 0";
    EXPECT_EQ(filter->getCount(servo_id_), 0) << "count != 0";
    EXPECT_EQ(filter->getDuty(servo_id_), 500) << "duty != 500";
    EXPECT_EQ(filter->getInvert(servo_id_), 1) << "invert != 1";

    filter->update(servo_id_, start + ros::Duration(0.5), duty, 300);
    filter->getServoPosition(servo_id_, position, is_valid);
    EXPECT_EQ(position, 300) << "position != 300";
    EXPECT_TRUE(is_valid) << "is_valid != true";
    EXPECT_EQ(filter->getRevolutions(servo_id_), 0) << "revolutions != 0";
    EXPECT_EQ(filter->getCount(servo_id_), 300) << "count != 300";
    EXPECT_EQ(filter->getDuty(servo_id_), 500) << "duty != 500";
    EXPECT_EQ(filter->getInvert(servo_id_), 1) << "invert != 1";

    filter->update(servo_id_, start + ros::Duration(1.0), duty, 600);
    filter->getServoPosition(servo_id_, position, is_valid);
    EXPECT_EQ(position, 600) << "position != 600";
    EXPECT_TRUE(is_valid) << "is_valid != true";
    EXPECT_EQ(filter->getRevolutions(servo_id_), 0) << "revolutions != 0";
    EXPECT_EQ(filter->getCount(servo_id_), 600) << "count != 600";
    EXPECT_EQ(filter->getDuty(servo_id_), 500) << "duty != 500";
    EXPECT_EQ(filter->getInvert(servo_id_), 1) << "invert != 1";

    filter->update(servo_id_, start + ros::Duration(1.5), duty, 900);
    filter->getServoPosition(servo_id_, position, is_valid);
    EXPECT_EQ(position, 900) << "position != 900";
    EXPECT_TRUE(is_valid) << "is_valid != true";
    EXPECT_EQ(filter->getRevolutions(servo_id_), 0) << "revolutions != 0";
    EXPECT_EQ(filter->getCount(servo_id_), 900) << "count != 900";
    EXPECT_EQ(filter->getDuty(servo_id_), 500) << "duty != 500";
    EXPECT_EQ(filter->getInvert(servo_id_), 1) << "invert != 1";

    filter->update(servo_id_, start + ros::Duration(2.0), duty, 1100);
    filter->getServoPosition(servo_id_, position, is_valid);
    EXPECT_EQ(position, 1100) << "position != 1100";
    EXPECT_TRUE(is_valid) << "is_valid != true";
    EXPECT_EQ(filter->getRevolutions(servo_id_), 0) << "revolutions != 0";
    EXPECT_EQ(filter->getCount(servo_id_), 1100) << "count != 1100";
    EXPECT_EQ(filter->getDuty(servo_id_), 500) << "duty != 500";
    EXPECT_EQ(filter->getInvert(servo_id_), 1) << "invert != 1";

    // filter->update(servo_id_, start + ros::Duration(2.5), duty, 900);
    // filter->getServoPosition(servo_id_, position, is_valid);
    // EXPECT_EQ(position, 900) << "position != 1100";
    // EXPECT_TRUE(is_valid) << "is_valid != false";
    // EXPECT_EQ(filter->getRevolutions(servo_id_), 0) << "revolutions != 0";
    // EXPECT_EQ(filter->getCount(servo_id_), 1400) << "count != 1100";
    // EXPECT_EQ(filter->getDuty(servo_id_), 500) << "duty != 500";
    // EXPECT_EQ(filter->getInvert(servo_id_), 1) << "invert != 1";
}


TEST_F(EncoderFilterPythonTestSuite, testGetPosition)
{
    std::unique_ptr<lx16a::LX16AEncoderFilter> filter(
        new lx16a::LX16AEncoderFilterPython(
            classifier_filename_,
            regressor_filename_,
            window_));
    filter->init();
    filter->add(servo_id_);

    int16_t position = 0;
    bool is_valid = false;
    filter->getServoPosition(servo_id_, position, is_valid);
    EXPECT_EQ(position, 0) << "position != 0";
    EXPECT_TRUE(is_valid) << "is_valid != true";
}

// Run tests
int main(int argc, char **argv)
{
    // Initialise the Python interpreter
    py::scoped_interpreter guard{};
    lx16a::addCmdArgsToSys(argc, argv);

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
