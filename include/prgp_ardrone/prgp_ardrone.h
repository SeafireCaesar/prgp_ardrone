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
 *  @brief The head file to program and test the ardrone.
 *  @details This is initially created and proposed by Chengqing Liu
 *  @version 0.9
 *  @author
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

#include <unistd.h>
#include <sstream>
#include <cmath>
#include <cstdio>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <algorithm>


#include <pthread.h> // for pthread_mutex_t

typedef enum
{
  TAG_TYPE_NONE             = 0,
  TAG_TYPE_SHELL_TAG        ,
  TAG_TYPE_ROUNDEL          ,
  TAG_TYPE_ORIENTED_ROUNDEL ,
  TAG_TYPE_STRIPE           ,
  TAG_TYPE_CAP              ,
  TAG_TYPE_SHELL_TAG_V2     ,
  TAG_TYPE_TOWER_SIDE       ,
  TAG_TYPE_BLACK_ROUNDEL    ,
  TAG_TYPE_NUM
} TAG_TYPE;

#define TAG_TYPE_MASK(tagtype) (  ((tagtype)==0)? 0 : (1<<((tagtype)-1)) )


#define  ARDRONE_DETECTION_COLOR_ORANGE_GREEN  1   /*!< Cameras detect orange-green-orange tags */
#define  ARDRONE_DETECTION_COLOR_ORANGE_YELLOW 2    /*!< Cameras detect orange-yellow-orange tags*/
#define  ARDRONE_DETECTION_COLOR_ORANGE_BLUE 3      /*!< Cameras detect orange-blue-orange tags */

/*
ros::NodeHandle ndh_;
ros::Duration ndPause;

//Publishers
ros::Publisher landPub;			//send landing commands
ros::Publisher takeoffPub;		//send takeoff commands
ros::Publisher drone_pub;		//send commands to AR.Drone
ros::Publisher cmdPub;			//To sen cnd to PiSwarm
ros::Publisher velPub;			//send cmd directly to cmd_vel topic of the ardrone_autonomy

//Subscribers
ros::Subscriber cmdSub;			//To get cnd from PiSwarm
ros::Subscriber tagSub;			//To get Tag detection result
ros::Subscriber currentPosSub;
ros::Subscriber imgSub;

ros::ServiceClient toggle_Cam_Srv;
*/

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
uint16_t flight_task_completed = 0;
uint16_t current_cam = 0; //0 is horizonal, 1 is vertical
uint16_t cnd_rev;
uint16_t target_tag;

#define COME_BACK 1
#define START 1



#endif /* PRGP_ARDRONE_INCLUDE_PRGP_ARDRONE_H_ */
