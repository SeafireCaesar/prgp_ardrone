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
 *  @file prgp_ardrone.h
 *  @brief The head file for prgp_ardrone package.
 *  @details The prgp_ardrone package and its structure and initial comments are created and tested by Chengqing Liu
 *  @version 1.0
 *  @author  , , , Chengqing Liu
 *  @date 24 July 2015
 *  @copyright BSD License.
 */

#ifndef PRGP_ARDRONE_INCLUDE_PRGP_ARDRONE_H_
#define PRGP_ARDRONE_INCLUDE_PRGP_ARDRONE_H_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Duration.h"

#include "ardrone_autonomy/Navdata.h"
#include "tum_ardrone/filter_state.h"

///sy   taking picture headers ########################
#include "cvd/image_io.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <iostream>

#include <unistd.h>
#include <sstream>
#include <cmath>
#include <cstdio>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <algorithm>
#include <pthread.h>

#define CLASS_STYLE /**< Macro to open the class style code */

#ifdef CLASS_STYLE

/** The main class for the prgp_ardrone package.
 */
class PRGPARDrone
{
private:
  ros::NodeHandle ndh_; /**< ROS node handle */
  ros::Duration ndPause; /**< The duration to pause the node */
  ros::Time pre_time; /**< The ROS time */

  //Publishers
  ros::Publisher landPub; /**< Publisher for sending the landing command directly to AR.Drone*/
  ros::Publisher takeoffPub; /**< Publisher for sending the takeoff command directly to AR.Drone */
  ros::Publisher drone_pub; /**< Publisher for sending flight command to AR.Drone by /tum_ardrone/com */
  ros::Publisher cmdPub; /**< Publisher for sending the command to Pi-Swarm by piswarm_com */
  ros::Publisher velPub; /**< Publisher for sending the command directly to AR.Drone by cmd_vel */

  //Subscribers
  ros::Subscriber cmdSub; /**< Subscriber to get the command from Pi-Swarm by piswarm_com */
  ros::Subscriber tagSub; /**< Subscriber to get the navdata, especially the tag result by /ardrone/navdata */
  ros::Subscriber currentPosSub; /**< Subscriber to get the current position of the AR.Drone */
  ros::Subscriber imgSub; /**< Subscriber to get the image from camera by /ardrone/image_raw */

  // services
  ros::ServiceClient toggleCamSrv; /**< Service client to send empty service to toggle the camera */
  ros::ServiceClient detecttypeSrv; /**< Service client to send empty service to change the detection configuration */

  //messages
  //std_msgs::String s; /**< Message for sending flight command to AR.Drone by /tum_ardrone/com*/
  //std::string c;

  std_msgs::String s_Pi; /**< Message for sending command to Pi-Swarm by piswarm_com*/
  std::string c_Pi;

  std_srvs::Empty toggle_srvs; /**< Service from client to server to toggle the camera */
  std_srvs::Empty detect_srvs; /**< Service from client to server to change the detection configuration */
  geometry_msgs::Twist velCmd; /**< Message for the cmd_vel topic to AR.Drone */

  static pthread_mutex_t send_CS;

  //variables
  double currentPos_x;
  double currentPos_y;
  bool start_flag; /**< The value will be true when AR.Drone get the recruiting command from Pi-Swarm */
  bool detected_flag; /**< The value will be true when AR.Drone detect the target tag during the flight */
  bool centering_flag; /**< The value will be true before AR.Drone centering the tag */
  bool picture_flag; /**< The value will be true before taking the picture */
  bool return_flag; /**< The value will be true before returning the Pi-Swarm back home */
  bool init_tag_det; /**< The value will be true to activate the tag detection at the initial stage of AR.Drone */
  bool init_detected_flag; /**< The value will be true when tag detected at the initial stage */
  bool home_tag_det; /**< The value will be true to activate the tag detection at the home stage of AR.Drone */
  bool home_detected_flag; /**< The value will be true when tag detected at home stage */
  uint16_t current_tag; /**< change when setTargetTag() is used, 0 for black_roundel, 1 for COCARDE */
  uint16_t tag_type; /**< 0 for black_roundel, 1 for COCARDE, 2 for mixed tag type (current_tag is 0) */

public:
  PRGPARDrone(void);
  ~PRGPARDrone(void);

  void run();

  // callbacks
  void piswarmCmdRev(const std_msgs::StringConstPtr str);
  void takePic(const sensor_msgs::ImageConstPtr img);
  void acquireTagResult(const ardrone_autonomy::Navdata &navdataReceived);
  void acquireCurrentPos(const tum_ardrone::filter_state& currentPos);

  //functions
  void sendCmdToPiswarm();
  void sendVelCmd();
  void takeOff();
  void land();
  void sendFlightCmd(std::string c);
  void toggleCam();
  void setTargetTag();
  bool initARDrone();
  void centeringTag();
  void flightToSearchTag();
  void flightToTarget();
  void flightToHome();
  void moveToPose(double x, double y, double z, double yaw);
};

#else //else for CLASS_STYLE

std_msgs::String s;
std::string c;

std_msgs::String s_Pi;
std::string c_Pi;

std_srvs::Empty toggle_srvs;
std_srvs::Empty detect_srvs;
geometry_msgs::Twist velCmd;

static pthread_mutex_t send_CS = PTHREAD_MUTEX_INITIALIZER;

float currentPos_x;
float currentPos_y;

bool tag_detected = false;

#endif//end of CLASS_STYLE

#endif /* PRGP_ARDRONE_INCLUDE_PRGP_ARDRONE_H_ */
