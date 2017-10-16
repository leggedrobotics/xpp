

// Bring in my package's API, which is what I'm testing
#include <gtest/gtest.h>
#include <xpp_vis/rviz_robot_builder.h>

// Declare a test
TEST(XppVis, testCase1)
{
  int i = 3;
  ASSERT_EQ(i,3);
  ASSERT_EQ(i,3);
}

// Declare another test
TEST(XppVis, testCase2)
{
  int i = 3;
  EXPECT_EQ(i,3);
  EXPECT_EQ(i,4);
}
