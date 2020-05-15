#include <gtest/gtest.h>
#include <ros/ros.h>

class HardwareEncoderTestSuite : public ::testing::Test
{
protected:
    ~HardwareEncoderTestSuite() override
    {
    }

    HardwareEncoderTestSuite()
    {
        // Load the test data.

    }

    void SetUp() override
    {        
    }

    void TearDown() override
    {        
    }
};


// Declare tests
TEST_F(HardwareEncoderTestSuite, testConstructor)
{
}

TEST_F(HardwareEncoderTestSuite, testUpdate)
{
}

// Run tests
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
