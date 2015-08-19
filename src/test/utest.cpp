#include "gtest/gtest.h"
#include "prgp_ardrone/prgp_ardrone.h"
#include <iostream>

TEST(CustomDroneTest, InitTest)
{
  PRGPARDrone drone;
  std::cout << "========== Start testing initARDrone() ==========" << std::endl;
  EXPECT_TRUE(drone.initARDrone());
}
TEST(CustomDroneTest, SallRangeSearchTest)
{
  PRGPARDrone drone;
  std::cout << "========== Start testing SmallRangeSearch() ==========" << std::endl;
  EXPECT_TRUE(drone.smallRangeSearch());
}
TEST(CustomDroneTest, SearchForTargetTag)
{
  PRGPARDrone drone;
  std::cout << "========== Start testing searchForTargetTag() ==========" << std::endl;
  EXPECT_TRUE(drone.searchForTargetTag());
}
TEST(CustomDroneTest, CenteringTagTest)
{
  PRGPARDrone drone;
  std::cout << "========== Start testing centeringTag(double current_height) ==========" << std::endl;
  EXPECT_FALSE(drone.centeringTag(-1));
  EXPECT_FALSE(drone.centeringTag(0));
  EXPECT_TRUE(drone.centeringTag(DESIRED_HEIGHT));
  EXPECT_TRUE(drone.centeringTag(DESIRED_HEIGHT + EXTRA_HEIGHT));
  EXPECT_FALSE(drone.centeringTag(100));
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "prgp_ardrone_test");
  return RUN_ALL_TESTS();
}
