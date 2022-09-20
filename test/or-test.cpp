#include <gtest/gtest.h>
#include <ros/package.h>
#include <ros/ros.h>

#include "or_tools_catkin/or_interface.h"

using namespace or_tools_catkin;


TEST(ORTests, SmallTask){
  EXPECT_TRUE(smallTask());
}



int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
