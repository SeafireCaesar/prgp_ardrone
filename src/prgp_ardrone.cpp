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
 *  @file prgp_ardrone.cpp
 *  @brief The source file for prgp_ardrone package.
 *  @details The prgp_ardrone package and its structure and initial comments are created and tested by Chengqing Liu.
 *  @version 1.0
 *  @author  Robert Evans, Shengsong Yang, Homero Silva, Chengqing Liu
 *  @date 24 July 2015
 *  @copyright BSD License.
 */
#include "prgp_ardrone/image.h"
#include <prgp_ardrone/prgp_ardrone.h>

pthread_mutex_t PRGPARDrone::send_CS = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t PRGPARDrone::pic_mt = PTHREAD_MUTEX_INITIALIZER;

/** Initialise the variables and paramaters.
 *  Initialise the ROS time, ROS Duration, Publishers, Subscribers, Service clients, Flags and so on.
 */
PRGPARDrone::PRGPARDrone()
{
  //variables in the class are initialized here.

  ndPause = ros::Duration(2, 0);
  pre_time = ros::Time::now();

  //if the topic cannot be resolved, try to change the topic below to "ndh_.resolveName("topic")"
  //Publishers
  landPub = ndh_.advertise<std_msgs::Empty>("/ardrone/land", 1);
  takeoffPub = ndh_.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
  drone_pub = ndh_.advertise<std_msgs::String>("/tum_ardrone/com", 50);
  cmdPub = ndh_.advertise<std_msgs::String>("piswarm_com", 1);
  velPub = ndh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  //Subscribers
  cmdSub = ndh_.subscribe("piswarm_com", 1, &PRGPARDrone::piswarmCmdRevCb, this);
  tagSub = ndh_.subscribe("/ardrone/navdata", 1, &PRGPARDrone::acquireTagResultCb, this);
  currentStateSub = ndh_.subscribe("/ardrone/predictedPose", 1, &PRGPARDrone::acquireCurrentStateCb, this);
  imgSub = ndh_.subscribe("/ardrone/image_raw", 10, &PRGPARDrone::takePicCb, this);
  image_cmd_pub = ndh_.advertise<std_msgs::Empty>("image_test", 10, this); ///
  image_cmd_sub = ndh_.subscribe("image_test", 10, &PRGPARDrone::imageCb, this); ///

  //Service client
  toggleCamSrv = ndh_.serviceClient<std_srvs::Empty>("/ardrone/togglecam", 1);
  detecttypeSrv = ndh_.serviceClient<std_srvs::Empty>("/ardrone/detecttype", 1);
  stopCmdAndHoverSrv = ndh_.serviceClient<std_srvs::Empty>("drone_autopilot/clearCommands", 1);

  //Variables
  start_flag = false;
  initialising_PTAM_flag = false;
  aligning_to_home_tag = false;
  detected_flag = false;
  centering_flag = false;
  picture_flag = false;
  return_flag = false;
  init_tag_det = false;
  home_tag_det = false;
  executing_command_flag = false;
  current_tag = 0;
  target_tag = 0;
  altitude = 0;
  reference_set = false;
  home = true;
  image_saved = false;

  altitude = 0; //Initialise the private variable in the Class definition will show warning.
  tag_x_coord = 0;
  tag_y_coord = 0;
  tag_orient = 0;
  tag_x_coord_h = 0;
  tag_y_coord_h = 0;
  tag_orient_h = 0;
  offset_x = 0;
  offset_y = 0;
}
/** Class destructor.
 *
 */
PRGPARDrone::~PRGPARDrone(void)
{
//  if (window != NULL)
//    delete (window);
  if (image != NULL)
    delete (image);
}

void PRGPARDrone::imageCb(const std_msgs::Empty msg) ///
{
  picture_flag = true;
} ///
/** Callback function for piswarm_com topic to get the command from the Pi-Swarm.
 *  Pi-Swarm send the recruiting command to the radio modem, the radio modem transfer the command to
 *  prgp_piswarmcom package. Then the prgp_piswarmcom package publish the command to the piswarm_com
 *  topic. This function get the command from the piswarm_com topic.
 */
void PRGPARDrone::piswarmCmdRevCb(const std_msgs::StringConstPtr str)
{

  ROS_INFO_STREAM(*str);
  ROS_INFO("%s\n", str->data.c_str());
  if (str->data == "r")
  {
    //default is used the black_roundel tag
    start_flag = true;
    target_tag = 0;
  }
  else if (str->data == "c")
  {
    setTargetTag(); //change to COCARDE tag
    start_flag = true;
    target_tag = 1;
  }
  else if (str->data == "m")
  {
    //mix tag, two black_roundel together, need not change the tag
    start_flag = true;
    target_tag = 2;
  }
}

/** Callback function for the /ardrone/image_raw topic to get the image from camera.
 *  When picture_flag become true, this function will start the taking picture function
 *  which get the image from the topic and process the image.
 */
void PRGPARDrone::takePicCb(const sensor_msgs::ImageConstPtr img)
{
  std::cout << "bang!" << std::endl;
  if (picture_flag == true)
  {
    std::cout << "stage 0 image == NULL?" << (image == NULL) << std::endl;
    if (image == NULL)
      image = new DroneImage(img, sensor_msgs::image_encodings::RGB8);
    else if (image->update())
    {
      picture_flag = false;
    }
  }
}

//    std::fstream image;
//    CVD::Image<CVD::Rgb<CVD::byte> > new_image;
//    picture_flag = false;
//    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::RGB8);
//    ROS_INFO("Before mutex");
//    pthread_mutex_lock(&pic_mt);
//    if (new_image.size().x != img->width || new_image.size().y != img->height)
//      new_image.resize(CVD::ImageRef(img->width, img->height));
//    memcpy(new_image.data(), cv_ptr->image.data, img->width * img->height * 3); ///sy cpy the image to mimFrameBW.data()
//    pthread_mutex_unlock(&pic_mt);
//    ROS_INFO("After mutex");
//    image.open("output.bmp", std::fstream::out);
//    CVD::img_save(new_image, image, CVD::ImageType::BMP);
//    ROS_INFO("Before window");
//    if (window != NULL)
//      delete (window);
//    window = new CVD::VideoDisplay(new_image.size());
//    ROS_INFO("After window");
//    glDrawPixels(new_image);
//    glFlush();
//    image.close();
//    image_saved = true;
//}

/** Callback function for /ardrone/navdata to get the navdata, especially the detection result.
 *  Getting the navdata from the topic and process the data. Then reporting the detection result
 *  for different stages, including the initial stage, flight stage and home stage of the AR.Drone.
 *  If target tag is detected, stop the flight command and hover the drone.
 */
void PRGPARDrone::acquireTagResultCb(const ardrone_autonomy::Navdata &navdataReceived)
{
  uint8_t i = 0;
  uint8_t j = 0;
  uint32_t x_coord[4] = {0, 0, 0, 0};
  uint32_t y_coord[4] = {0, 0, 0, 0};
  uint32_t width[4] = {0, 0, 0, 0};
  float orient[4] = {0, 0, 0, 0};

  detected_flag = false;
  detected_flag_h = false;

  altitude = navdataReceived.altd / 1000.0;

  if (navdataReceived.tags_count > 0)
  {
    if (home)
    {
      for (i = 0; i < navdataReceived.tags_count; i++)
      {
        if (navdataReceived.tags_type[i] == tag_v)
        {
          detected_flag = true;
          tag_x_coord = navdataReceived.tags_xc[i];
          tag_y_coord = navdataReceived.tags_yc[i];
          tag_orient = navdataReceived.tags_orientation[i];
        }

        if (navdataReceived.tags_type[i] == tag_h)
        {
          detected_flag_h = true;
          tag_x_coord_h = navdataReceived.tags_xc[i];
          tag_y_coord_h = navdataReceived.tags_yc[i];
          tag_orient_h = navdataReceived.tags_orientation[i];
        }
      }
    }
    else
    {
      j = 0;
      for (i = 0; i < navdataReceived.tags_count; i++)
      {
        if (navdataReceived.tags_type[i] == tag_v)
        {
          x_coord[j] = navdataReceived.tags_xc[i];
          y_coord[j] = navdataReceived.tags_yc[i];
          orient[j] = navdataReceived.tags_orientation[i];
          width[j] = navdataReceived.tags_width[i];
          j++;
        }
      }
      switch (target_tag)
      {
        case 0:

          if (j == 1)
          {
            if (width[0] > 45)
            {
              detected_flag = true;
              tag_x_coord = x_coord[0];
              tag_y_coord = y_coord[0];
              tag_orient = orient[0];
            }
          }
          else if (j == 2)
          {

            if ((width[1] - width[0]) > 9)
            {
              detected_flag = true;
              tag_x_coord = x_coord[1];
              tag_y_coord = y_coord[1];
              tag_orient = orient[1];
            }
            else if ((width[0] - width[1]) > 9)
            {
              detected_flag = true;
              tag_x_coord = x_coord[0];
              tag_y_coord = y_coord[0];
              tag_orient = orient[0];
            }
          }
          else if (j == 3)
          {

            if ((width[0] > width[1]) && (width[0] > width[2]))
            {
              detected_flag = true;
              tag_x_coord = x_coord[0];
              tag_y_coord = y_coord[0];
              tag_orient = orient[0];
            }
            if ((width[1] > width[0]) && (width[1] > width[2]))
            {
              detected_flag = true;
              tag_x_coord = x_coord[1];
              tag_y_coord = y_coord[1];
              tag_orient = orient[1];
            }
            if ((width[2] > width[0]) && (width[2] > width[1]))
            {
              detected_flag = true;
              tag_x_coord = x_coord[2];
              tag_y_coord = y_coord[2];
              tag_orient = orient[2];
            }
          }
          else
          {
            ROS_INFO("Tag Number is error!");
          }

          break;
        case 1:

          if (j == 1)
          {
            detected_flag = true;
            tag_x_coord = x_coord[0];
            tag_y_coord = y_coord[0];
            tag_orient = orient[0];
          }
          else
          {
            ROS_INFO("Tag Number is error!");
          }
          break;
        case 2:
          if (j == 1)
          {
            if (width[0] < 40)
            {
              detected_flag = true;
              tag_x_coord = x_coord[0];
              tag_y_coord = y_coord[0];
              tag_orient = orient[0];
            }
          }
          else if (j == 2)
          {

            if ((width[1] - width[0]) < 5)
            {
              detected_flag = true;
              tag_x_coord = x_coord[1];
              tag_y_coord = y_coord[1];
              tag_orient = orient[1];
            }
          }
          else if (j == 3)
          {

            if ((width[0] > width[1]) && (width[0] > width[2]))
            {
              detected_flag = true;
              tag_x_coord = x_coord[1];
              tag_y_coord = y_coord[1];
              tag_orient = orient[1];
            }
            if ((width[1] > width[0]) && (width[1] > width[2]))
            {
              detected_flag = true;
              tag_x_coord = x_coord[0];
              tag_y_coord = y_coord[0];
              tag_orient = orient[0];
            }
            if ((width[2] > width[0]) && (width[2] > width[1]))
            {
              detected_flag = true;
              tag_x_coord = x_coord[0];
              tag_y_coord = y_coord[0];
              tag_orient = orient[0];
            }
          }
          else
          {
            ROS_INFO("Tag Number is error!");
          }
          break;
        default:
          break;
      }
    }
  }
  ROS_INFO("TagResultCb is completed.");
}

/** Callback function for /ardrone/predictedPose to get the current state of AR.Drone.
 *  Getting the current state from the topic and process it for different requirements.
 *  If the PTAM is lost, stop the flight command and hover the drone in order to save it.
 */
void PRGPARDrone::acquireCurrentStateCb(const tum_ardrone::filter_state &currentState)
{
  currentPos_x = currentState.x - offset_x;
  currentPos_y = currentState.y - offset_y;
//ROS_INFO("x:%.2f(%.2f) y:%.2f (%.2f)", currentPos_x,offset_x,currentPos_y,offset_y);
  if (fabs(currentPos_x) > 1.6 || fabs(currentPos_y) > 1.6)
  {
//ROS_INFO("outside home x:%.2f y:%.2f", currentPos_x, currentPos_y);
    home = false;
  }
  else
  {
    home = true;
    if (detected_flag)
      ROS_INFO("HOME AND DETECTED");
  }
  if (currentState.ptamState == currentState.PTAM_LOST)
  {
    stopCmdAndHover();
  }
}

/** Sending the command to the Pi-Swarm by the topic piswarm_com.
 *  The returning command is published to the topic. The prgp_piswarmcom package get the command
 *  and send to the radio modem. Then the radio modem will send the command to the Pi-Swarm.
 */
void PRGPARDrone::sendCmdToPiswarm()
{
  std_msgs::String s_Pi; /**< Message for sending command to Pi-Swarm by piswarm_com*/
  std::string c_Pi;
  c_Pi = "b";
  s_Pi.data = c_Pi.c_str();
  cmdPub.publish(s_Pi);
}

/** Sending the command directly to the ardrone_autonomy package by cmd_vel topic.
 *  Sending the command to control the yaw, gaz, pitch, roll and other paramaters.
 */
void PRGPARDrone::sendVelCmd() ///sy TODO not used
{
  velCmd.angular.z = 0; // -cmd.yaw;
  velCmd.linear.z = 0; //cmd.gaz;
  velCmd.linear.x = 0; //-cmd.pitch;
  velCmd.linear.y = 0; //-cmd.roll;
  velCmd.angular.x = velCmd.angular.y = 0; //gui->useHovering ? 0 : 1;
  velPub.publish(velCmd);
}

/** Sending the takeoff command directly to the ardrone_autonomy package.
 */
void PRGPARDrone::takeOff() ///sy TODO not used
{
  takeoffPub.publish(std_msgs::Empty());
  ROS_INFO("Takeoff");

}

/** Sending the landing command directly to the ardrone_autonomy package.
 */
void PRGPARDrone::land()
{
  landPub.publish(std_msgs::Empty());
  ROS_INFO("Land");
}

/** Sending the flight command to the tum_ardrone package by the topic /tum_ardrone/com.
 */
void PRGPARDrone::sendFlightCmd(std::string c)
{
  std_msgs::String s; /**< Message for sending flight command to AR.Drone by /tum_ardrone/com*/
  s.data = c.c_str();
  pthread_mutex_lock(&send_CS);
  drone_pub.publish(s);
  pthread_mutex_unlock(&send_CS);
  ROS_INFO("%s", c.c_str());
}

/** Moving ARDrone to a certain pose.
 *
 */
void PRGPARDrone::moveToPose(double x, double y, double z, double yaw = 0)
{
  char buff[100];
  sprintf(buff, "c goto %.2f %.2f %.2f %.2f", x, y, z, yaw);
  std::string c = buff;
  sendFlightCmd(c);
}

/** Stop the current flight command and hover the ARDrone.
 *  The function use a separate service defined in the tum_ardrone package to
 *  clear the command queue (gaz, pitch, roll, yaw) in order to stop the current
 *  flight command and stop the AR.Drone by delete the currentKI which is the instance
 *  to master a command in tum_ardrone. This method with service is better than
 *  simply sending "c clearCommands" to the /tum_ardrone/com topic as this topic is used
 *  by a lot of nodes (several publishers and subscribers).
 */
void PRGPARDrone::stopCmdAndHover()
{
  double tag_x = currentPos_x;
  double tag_y = currentPos_y;
  stopCmdAndHoverSrv.call(stopCmd_srvs);
  ROS_INFO("Command is cleared.");
  ndPause.sleep();
  moveToPose(tag_x, tag_y, 0, 0);
}

/** Moving ARDrone by a distance and angle from its current pose.
 *
 */
void PRGPARDrone::moveBy(double x, double y, double z, double yaw = 0)
{
  char buff[100];
  sprintf(buff, "c moveBy %.2f %.2f %.2f %.2f", x, y, z, yaw);
  std::string c = buff;
  sendFlightCmd(c);
}

/** Toggling the camera during the flight.
 *  The default camera is the front camera. When toggling happens, the camera will change to the
 *  vertical. And when toggling again, the camera will return to the front one.
 */
void PRGPARDrone::toggleCam()
{
  toggleCamSrv.call(toggle_srvs);
  ROS_INFO("toggle the camera");
}

/** Reconfiguring the detection during the flight.
 *  The default detection type is the black_roundel. Running this function will change the detection
 *  to COCARDE. Running again will change the detection back to black_roundel.
 */
void PRGPARDrone::setTargetTag()
{
  detecttypeSrv.call(detect_srvs);
  ROS_INFO("change the detect type");
  current_tag = (current_tag + 1) % 2;
}

bool PRGPARDrone::smallRangeSearch()
{
  if (detected_flag == true)
  {
    ROS_INFO("TAG in sight, small range search aborted");
    return true;
  }
  double small_distance = 0;

  if (home)
  {
    small_distance = 0.5;
  }
  else
  {
    small_distance = 0.5;
  }

  double command_list[8][4] = { {0, -small_distance, 0, 0}, //go backwards
      {-small_distance, 0, 0, 0}, //go left
      {0, small_distance, 0, 0}, //go forward
      {0, small_distance, 0, 0}, //go forward
      {small_distance, 0, 0, 0}, //go right
      {small_distance, 0, 0, 0}, //go right
      {0, -small_distance, 0, 0}, //go backwards
      {0, -small_distance, 0, 0}, //go backwards
      };

  int i = 0;
  while (detected_flag == false && i < 8)
  {
    ROS_INFO("search for tag");

    moveBy(command_list[i][0], command_list[i][1], command_list[i][2], command_list[i][3]);

    ndPause.sleep();
    ndPause.sleep();
    ndPause.sleep(); //TODO
    ros::spinOnce();
    i++;
  }

  if (detected_flag == true)
  {
    ROS_INFO("Small range search finished");
    return true;
  }
  else
  {
    ROS_INFO("Small range search failed");
    return false;
  }
}

/** Initialise the ARDrone when it starts.
 *  Initialise the PTAM and set the reference point.
 */
bool PRGPARDrone::initARDrone()
{

  ndPause.sleep(); //TODO
  ndPause.sleep();
  ndPause.sleep();
  ndPause.sleep();

  sendFlightCmd("c start");
  sendFlightCmd("c takeoff");
  sendFlightCmd("c autoTakeover 500 800 4000 0.5");
  sendFlightCmd("c setMaxControl 1"); //set AR.Drone speed limit
  sendFlightCmd("c setInitialReachDist 0.2");
  sendFlightCmd("c setStayWithinDist 0.5");
// stay 1 seconds
  sendFlightCmd("c setStayTime 2");
//PTAM
  sendFlightCmd("c lockScaleFP");

  sendFlightCmd("c setReference $POSE$");
  ros::spinOnce();
  offset_x = currentPos_x + offset_x;
  offset_y = currentPos_y + offset_y;

  ndPause.sleep();
  ndPause.sleep();
  ndPause.sleep();
  ndPause.sleep();
  ros::spinOnce();

  ROS_INFO("Planned change in alt: %f. Current altd: %f", (DESIRED_HEIGHT - altitude), altitude);
  moveBy(0.0, 0.0, (DESIRED_HEIGHT - altitude), 0.0);

  sendFlightCmd("c setReference $POSE$");
  ros::spinOnce();
  offset_x = currentPos_x + offset_x;
  offset_y = currentPos_y + offset_y;
  moveToPose(0.0, 0.0, EXTRA_HEIGHT, 0);

  ndPause.sleep();
  ndPause.sleep();
  ndPause.sleep();
  ndPause.sleep();
  ndPause.sleep();
  ndPause.sleep();

  ros::spinOnce();

  if (centeringTag(DESIRED_HEIGHT + EXTRA_HEIGHT))
  {
    moveBy(0, 0, -EXTRA_HEIGHT, 0);
    ros::spinOnce();
    sendFlightCmd("c setReference $POSE$");
    offset_x = currentPos_x + offset_x;
    offset_y = currentPos_y + offset_y;
    return true;
  }
  else
  {
    ROS_INFO("Unable to centre on home tag");
    return false;
  }
  return false;
}

/** Flight and searching the target tag.
 *  Sending the flight commands to control the flight.
 */ //Rob# This function name is also a little unclear. SearchForTargetTag
bool PRGPARDrone::searchForTargetTag()
{
#undef NUM_OF_CMD
#define NUM_OF_CMD 5
//  double command_list_search[NUM_OF_CMD][4] = { {0.8, 0, 0, 0}, {1.6, 0, 0, 0}, {2.4, 0, 0, 0}, /*{2.4, -0.81, 0, 0}, {3.2, -0.81,
//   0, 0},*/
//                                               {3.2, 0, 0, 0}, {3.2, 0.81, 0, 0}, {3.2, 1.62, 0, 0}, {3.2, 2.43, 0, 0},
//                                               {3.2, 3.24, 0, 0}, {3.2, 4.05, 0, 0}, {3.2, 4.86, 0, 0},
//                                               {3.2, 5.67, 0, 0}, {4, 5.67, 0, 0}, {4, 4.86, 0, 0}, {4, 4.05, 0, 0}, {
//                                                   4, 3.24, 0, 0},
//                                               {4, 2.43, 0, 0}, {4, 1.62, 0, 0}, {4, 0.81, 0, 0}, {4, 0, 0, 0}, /*{4, -0.81, 0, 0}, {4.8, -0.81, 0, 0}, */
//                                               {4.8, 0, 0, 0}, {4.8, 0.81, 0, 0}, {4.8, 1.62, 0, 0}, {4.8, 2.43, 0, 0},
//                                               {4.8, 3.24, 0, 0}, {4.8, 4.05, 0, 0}, {4.8, 4.86, 0, 0},
//                                               {4.8, 5.67, 0, 0}, {5.6, 5.67, 0, 0}, {5.6, 4.86, 0, 0},
//                                               {5.6, 4.05, 0, 0}, {5.6, 3.26, 0, 0}, {5.6, 2.43, 0, 0},
//                                               {5.6, 1.62, 0, 0}, {5.6, 0.81, 0, 0}, {5.6, 0, 0, 0}, /*{5.6,
//                                                -0.81,
//                                                0, 0},*/
//                                               {5.6, 0, 0, 0}, {4.8, 0, 0, 0}, {4, 0, 0, 0}, {3.2, 0, 0, 0}, {2.4, 0, 0,
//                                                                                                              0},
//                                               {1.6, 0, 0, 0}, {0.8, 0, 0, 0}, {0, 0, 0, 0}};
  double command_list_search[NUM_OF_CMD][4] = { {-2, 0, 0, 0}, {-2, 3, 0, 0}, {-2, 0, 0, 0}, {-2, 3, 0, 0},
                                               {-2, 0, 0, 0}, /* {-2, 3, 0, 0}, {-2, 0, 0, 0}, {-2, 3, 0, 0}, {-2, 0, 0, 0}, {-2, 3, 0, 0}, {-2, 0, 0, 0}, {-2, 3, 0, 0},
   {-2, 0, 0, 0}, {-2, 3, 0, 0}, {-2, 0, 0, 0}, {-2, 3, 0, 0}, {-2, 0, 0, 0}, {-2, 3, 0, 0}, {-2, 0, 0, 0}, {-2, 3, 0, 0}, {-2, 0, 0, 0}, {-2, 3, 0, 0}, {-2, 0, 0, 0}, {-2, 3, 0, 0},
   {-2, 0, 0, 0}, {-2, 3, 0, 0}, {-2, 0, 0, 0}, {-2, 3, 0, 0}, {-2, 0, 0, 0}, {-2, 3, 0, 0}, {-2, 0, 0, 0}, {-2, 3, 0, 0}, {-2, 0, 0, 0}, {-2, 3, 0, 0}, {-2, 0, 0, 0}, {-2, 3, 0, 0},
   {-2, 0, 0, 0}, {-2, 3, 0, 0}, {-2, 0, 0, 0}, {-2, 3, 0, 0}, {-2, 0, 0, 0}, {-2, 3, 0, 0}, {-2, 0, 0, 0}, {-2, 3, 0, 0}, {-2, 0, 0, 0}, {-2, 3, 0, 0}, {-2, 0, 0, 0}, {-2, 3, 0, 0}
   */};
  int i = 0;
  while (i < NUM_OF_CMD)
  {
    moveToPose(command_list_search[i][0], command_list_search[i][1], command_list_search[i][2],
               command_list_search[i][3]);
    i++;
  }
  ros::spinOnce();
  while (home) ///sy wait till drone is outside of home
  {
    ROS_INFO("Drone is at home");
    ros::spinOnce();
  }
  while (!detected_flag && !home) ///sy wait till tag is detected or come back to home
  {
    ROS_INFO("Drone is outside home searching");
    ros::spinOnce();
  }

  if (detected_flag && !home)
  {
    ROS_INFO("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
    stopCmdAndHover();
    ndPause.sleep(); //TODO
    ndPause.sleep();
    ndPause.sleep();
    ros::spinOnce();

    if (centeringTag(DESIRED_HEIGHT))
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    ROS_INFO("Search complete but tag not found");
    return false;
  }
  return false;
}

/** Centering the target tag when the target tag is detected.
 *
 */
bool PRGPARDrone::centeringTag(double current_height)
{
//Update tag information
  ros::spinOnce();
  if (smallRangeSearch())
  {
//This conversion is for a height of 200cm only
    float x_move = (float)tag_x_coord - 500;
    x_move = x_move * current_height / 1070;
//x_move = x_move * current_height / 1200;
    float y_move = (float)tag_y_coord - 500;
    y_move = y_move * current_height / 1940 * -1;
//y_move = y_move * current_height / 2300 * -1;
    float angle_to_turn = 0;
    if (home == true)
    {
      angle_to_turn = 180 - tag_orient;
    }
    else
    {
      angle_to_turn = 0;
    }
//Error handling
    if (x_move > 1 || y_move > 1 || x_move < -1 || y_move < -1 || angle_to_turn > 200 || angle_to_turn < -200)
    {
      ROS_WARN("Centring move command out of allowed range");
      return false;
    }
    else
    {
      ROS_INFO("x coord is: %d, y coord is: %d", tag_x_coord, tag_y_coord);
      ROS_INFO("Moving so Tag is at centre");
      moveBy(x_move, y_move, 0.0, angle_to_turn);
      ndPause.sleep();
      ndPause.sleep();
      ndPause.sleep();
      ndPause.sleep();
      ndPause.sleep(); //TODO
      ros::spinOnce();
    }
    if (detected_flag == true)
    {
      ROS_INFO("Drone centred above Tag");
      return true;
    }
    else
    {
      ROS_INFO("TAG lost during centring. Centring failed");
      return false;
    }
  }
  else
  {
    ROS_INFO("Tag not detected so unable to centre");
    return false;
  }
  ROS_WARN("Code should not get here");
  return false;
}

/** Send commands to fly home.
 *
 */
void PRGPARDrone::flightToHome() //TODO
{
  ROS_INFO("Current position is : %0.2f, %0.2f", (currentPos_x), (currentPos_y));
}

/** The main running loop for the prgp_ardrone package.
 *  Getting the command from Pi-Swarm to start the AR.Drone. Then flight to the target. centering
 *  the target, taking the picture, returning home and send command to return the Pi-Swarm. All the
 *  functions are organised by the flags (true and false).
 */
void PRGPARDrone::run()
{
  ROS_INFO("Starting running");
  if (ros::ok())
  {
    while (1)
    {
      ndPause.sleep();
      std::cout << picture_flag << std::endl;
      ros::spinOnce();
    }
    if (!initARDrone())
    {
      ROS_INFO("Drone initialisation failed");
      sendFlightCmd("c land"); ///sy TODO land method
      return;
    }
    setTargetTag();
    if (!searchForTargetTag()) ///sy exit till centred or search failed
    {
      ROS_INFO("Drone search failed");
      sendFlightCmd("c land");
      return;
    }
    toggleCam();
    ndPause.sleep();
//    ros::spinOnce(); //
    picture_flag = true;
    if (picture_flag)
    {
      ROS_INFO("picture flag changed");

      ros::spinOnce();
    }
//    while(image_saved == false){
//      ros::spinOnce();
//    }
///sy takePicCb
    ndPause.sleep();
    sendCmdToPiswarm(); ///sy piswarm back
    toggleCam();
    while (1)
      ;
    flightToHome(); ///sy TODO make sure method returns only when it's home
    land(); ///sy TODO "c land"?
  }
}
//  while(ros::ok())
//  {
//    ROS_INFO("A new spin begins!");
//    ndPause.sleep();
//
//    /*//for testing
//     if(picture_flag == false)
//    {
//    	toggleCam();
//    	sendCmdToPiswarm();
//    }*/
//    /*
//    if(start_flag == true){
//    	flightToTarget();
//    }
//    if(detected_flag == true)
//	{
//    	centeringTag();
//	}
//	if(centering_flag == true)
//	{
//	  toggleCam(); //change camera to vertical
//	  picture_flag = true; //the call back will store and show the picture
//	}
//	if(return_flag == true)
//	{
//      sendCmdToPiswarm();
//      flightToHome();
//	}
//    //Rob #It might be better to put the spin at the start of the spin.
//     *
//     */
//   ros::spinOnce();
//  }

/** main function of the prgp_ardrone package.
 *  create the ROS node, define the instance of the PRGPARDrone class. Calling the running loop.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "prgp_ardrone"); //Create node
  ROS_INFO("Started prgp_ardrone Node. Hi from ARE 2014/15");

  PRGPARDrone PRGPARDrone;
  PRGPARDrone.run();

  return 0;
}

