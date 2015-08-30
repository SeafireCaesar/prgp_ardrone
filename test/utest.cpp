/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2015, University of York Robotics Laboratory (YRL).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the following
 *   disclaimer in the documentation and/or other materials provided
 *   with the distribution.
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 *  @file utest.cpp
 *  @brief The unit testing file for prgp_ardrone package.
 *  @details This unit testing is using google testing framework. The file lists some test cases for
 *  initARDrone(), smallRangeSearch(), and CenteringTag(double) methods of the PRGPARdrone class.
 *  The searchForTargetTag() method has been commented out because of the detected_flag and home
 *  flags will never be set to desired value while testing, it might get stuck in the while loop.
 *  @author Shengsong Yang
 *  @date 18 Aug 2015
 *  @copyright BSD License.
 */

#include "gtest/gtest.h"
#include "prgp_ardrone/prgp_ardrone.h"
#include <iostream>

TEST(CustomDroneTest, InitTest)
{
  PRGPARDrone drone;
  std::cout << "========== Start testing initARDrone() ==========" << std::endl;
  EXPECT_TRUE(drone.initARDrone());
}
TEST(CustomDroneTest, SmallRangeSearchTest)
{
  PRGPARDrone drone;
  std::cout << "========== Start testing smallRangeSearch() ==========" << std::endl;
  EXPECT_TRUE(drone.smallRangeSearch());
}
//TEST(CustomDroneTest, SearchForTargetTag)
//{
//  PRGPARDrone drone;
//  std::cout << "========== Start testing searchForTargetTag() ==========" << std::endl;
//  EXPECT_TRUE(drone.searchForTargetTag());
//  ///sy this test will keep running till the tag's found which could never be done solely in code
//}
TEST(CustomDroneTest, CenteringTagProperHeightTest)
{
  PRGPARDrone drone;
  std::cout << "========== Start testing centeringTag(double current_height) with proper height values==========" << std::endl;
  EXPECT_TRUE(drone.centeringTag(DESIRED_HEIGHT));
  EXPECT_TRUE(drone.centeringTag(DESIRED_HEIGHT + EXTRA_HEIGHT));
}
TEST(CustomDroneTest, CenteringTagWrongHeightTest)
{
  PRGPARDrone drone;
  std::cout << "========== Start testing centeringTag(double current_height) with wrong height values==========" << std::endl;
  EXPECT_FALSE(drone.centeringTag(-1));
  EXPECT_FALSE(drone.centeringTag(0));
  EXPECT_FALSE(drone.centeringTag(100));
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "prgp_ardrone_test");
  return RUN_ALL_TESTS();
}
