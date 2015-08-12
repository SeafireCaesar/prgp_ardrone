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
 *  @author  , , , Chengqing Liu
 *  @date 24 July 2015
 *  @copyright BSD License.
 */

#include <prgp_ardrone/prgp_ardrone.h>

#ifdef CLASS_STYLE
pthread_mutex_t PRGPARDrone::send_CS = PTHREAD_MUTEX_INITIALIZER;

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
  tag_type = 0;
  altitude = 0;
  reference_set = false;
  home = true;

}

PRGPARDrone::~PRGPARDrone(void)
{
  //do not write anything here
}

/** Callback function for piswarm_com topic to get the command from the Pi-Swarm.
 *  Pi-Swarm send the recruiting command to the radio modem, the radio modem transfer the command to
 *  prgp_piswarmcom package. Then the prgp_piswarmcom package publish the command to the piswarm_com
 *  topic. This function get the command from the piswarm_com topic.
 */
void PRGPARDrone::piswarmCmdRevCb(const std_msgs::StringConstPtr str)
{

  ROS_INFO_STREAM(*str);
  ROS_INFO("%s\n", str->data.c_str());
  std::string k = str->data.substr(0, 1);
  ROS_INFO("%s\n", k.c_str());
  if (str->data.c_str() == "r")
  {
    //default is used the black_roundel tag
    start_flag = true;
    tag_type = 0;
  }
  else if (str->data.c_str() == "c")
  {
    setTargetTag(); //change to COCARDE tag
    start_flag = true;
    tag_type = 1;
  }
  else if (str->data.c_str() == "m")
  {
    //mix tag, two black_roundel together, need not change the tag
    start_flag = true;
    tag_type = 2;
  }
}

/** Callback function for the /ardrone/image_raw topic to get the image from camera.
 *  When picture_flag become true, this function will start the taking picture function
 *  which get the image from the topic and process the image.
 */
void PRGPARDrone::takePicCb(const sensor_msgs::ImageConstPtr img)
{

  std::fstream image;
  CVD::Image<CVD::Rgb<CVD::byte> > new_image;
  if (picture_flag == true) ///sy TODO change the flag
  {
    picture_flag = false;
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::RGB8);

    if (new_image.size().x != img->width || new_image.size().y != img->height)
      new_image.resize(CVD::ImageRef(img->width, img->height));
    memcpy(new_image.data(), cv_ptr->image.data, img->width * img->height); ///sy cpy the image to mimFrameBW.data()
    //    newImageAvailable = true;

    image.open("output.bmp", std::fstream::out);
    std::cout << "****************** Printing Image ******************" << std::endl;
    CVD::img_save(new_image, image, CVD::ImageType::BMP);
    std::cout << "****************************************************" << std::endl;
    image.close();
//    image_saved = true;

    //store;
    //show;
//	centering_flag = false;
//    return_flag = true;
//    picture_flag = false;
//    toggleCam(); //change the camera back

  }
}

/** Callback function for /ardrone/navdata to get the navdata, especially the detection result.
 *  Getting the navdata from the topic and process the data. Then reporting the detection result
 *  for different stages, including the initial stage, flight stage and home stage of the AR.Drone.
 *  If target tag is detected, stop the flight command and hover the drone.
 */
void PRGPARDrone::acquireTagResultCb(const ardrone_autonomy::Navdata &navdataReceived)
{
  altitude = navdataReceived.altd / 1000.0;
  if (navdataReceived.tags_count > 0)
  {
    navdataReceived.tags_type[0];
    tag_x_coord = navdataReceived.tags_xc[0];
    tag_y_coord = navdataReceived.tags_yc[0];
    tag_orient = navdataReceived.tags_orientation[0];
    navdataReceived.tags_type[1];
    navdataReceived.tags_xc[1];
    navdataReceived.tags_yc[1];

    if (init_tag_det == true)
    {
      init_detected_flag = true;
    }
    else if (home_tag_det == true)
    {
      home_detected_flag = true;
    }
    else if (tag_type < 2)
    {
      detected_flag = true;
      start_flag = false;
      //stopCmdAndHover();
    }
    else if (2 == tag_type)
    {
      if ((navdataReceived.tags_count == 2)
          && (fabs(navdataReceived.tags_orientation[0] - navdataReceived.tags_orientation[0]) < 20))
      {
        detected_flag = true;
        start_flag = false;
        //stopCmdAndHover();
      }
    }
  }
  else
  {
    init_detected_flag = false;
    home_detected_flag = false;
    detected_flag = false;
  }
}

/** Callback function for /ardrone/predictedPose to get the current state of AR.Drone.
 *  Getting the current state from the topic and process it for different requirements.
 *  If the PTAM is lost, stop the flight command and hover the drone in order to save it.
 */
void PRGPARDrone::acquireCurrentStateCb(const tum_ardrone::filter_state &currentState)
{
  currentPos_x = currentState.x + offset_x;
  currentPos_y = currentState.y + offset_y;

  if (fabs(currentPos_x) > 1.6 || fabs(currentPos_y) > 1.6)
  {

    ROS_INFO("home = false x: %.2f y: %.2f",currentPos_x,currentPos_y);
    home = false;
  }
  else
  {
    home = true;
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
void PRGPARDrone::sendVelCmd()
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
void PRGPARDrone::takeOff()
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
  moveToPose(tag_x,tag_y,DESIRED_HEIGHT,0);
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
    ndPause.sleep();
    ros::spinOnce();
    i++;
  }

  if (detected_flag == true)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/** Initialise the ARDrone when it starts.
 *  Initialise the PTAM and set the reference point.
 */
bool PRGPARDrone::initARDrone()
{

  ndPause.sleep();
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
  offset_x = currentPos_x;
  offset_y = currentPos_y;

  ndPause.sleep();
  ndPause.sleep();
  ndPause.sleep();
  ndPause.sleep();

  ros::spinOnce();

  ROS_INFO("Planned change in alt: %f. Current altd: %f", (DESIRED_HEIGHT - altitude), altitude);
  moveBy(0.0, 0.0, (DESIRED_HEIGHT - altitude), 0.0);

  sendFlightCmd("c setReference $POSE$");
  offset_x = currentPos_x;
  offset_y = currentPos_y;
  ros::spinOnce();
  std::cout << "%%%%%%%%%%%" << std::endl;
  std::cout << currentPos_x << std::endl;
  std::cout << currentPos_y <<std::endl;
  moveToPose(0.0, 0.0, EXTRA_HEIGHT, 0);

  ndPause.sleep();
  ndPause.sleep();
  ndPause.sleep();
  ndPause.sleep();
  ndPause.sleep();
  ndPause.sleep();

  ros::spinOnce();

  if (1)//(smallRangeSearch() == true) ///sy small range search is part of centering
  {
    if (centeringTag(DESIRED_HEIGHT + EXTRA_HEIGHT))
    {
      moveBy(0, 0, -EXTRA_HEIGHT, 0);
      sendFlightCmd("c setReference $POSE$");
      return true;
    }
    else
    {
      ROS_INFO("Unable to centre on home tag");
      return false;
    }
  }
  else
  {
    ROS_INFO("Tag not detected: Centering failed");
    sendFlightCmd("c land");
    return false;
  }
  return true;	///sy default value should be false
}

/** Flight and searching the target tag.
 *  Sending the flight commands to control the flight.
 */ //Rob# This function name is also a little unclear. SearchForTargetTag
void PRGPARDrone::flightToSearchTag()
{
  double command_list_search[48][4] = { {0.8, 0, 0, 0}, {1.6, 0, 0, 0}, {2.4, 0, 0, 0}, /*{2.4, -0.81, 0, 0}, {3.2, -0.81,
                                                                                                             0, 0},*/
                                       {3.2, 0, 0, 0}, {3.2, 0.81, 0, 0}, {3.2, 1.62, 0, 0}, {3.2, 2.43, 0, 0}, {3.2,
                                                                                                                 3.24,
                                                                                                                 0, 0},
                                       {3.2, 4.05, 0, 0}, {3.2, 4.86, 0, 0}, {3.2, 5.67, 0, 0}, {4, 5.67, 0, 0}, {4,
                                                                                                                  4.86,
                                                                                                                  0, 0},
                                       {4, 4.05, 0, 0}, {4, 3.24, 0, 0}, {4, 2.43, 0, 0}, {4, 1.62, 0, 0}, {4, 0.81, 0,
                                                                                                            0},
                                       {4, 0, 0, 0}, /*{4, -0.81, 0, 0}, {4.8, -0.81, 0, 0}, */{4.8, 0, 0, 0}, {4.8, 0.81,
                                                                                                            0, 0},
                                       {4.8, 1.62, 0, 0}, {4.8, 2.43, 0, 0}, {4.8, 3.24, 0, 0}, {4.8, 4.05, 0, 0}, {
                                           4.8, 4.86, 0, 0},
                                       {4.8, 5.67, 0, 0}, {5.6, 5.67, 0, 0}, {5.6, 4.86, 0, 0}, {5.6, 4.05, 0, 0}, {
                                           5.6, 3.26, 0, 0},
                                       {5.6, 2.43, 0, 0}, {5.6, 1.62, 0, 0}, {5.6, 0.81, 0, 0}, {5.6, 0, 0, 0}, /*{5.6,
                                                                                                                 -0.81,
                                                                                                                 0, 0},*/
                                       {5.6, 0, 0, 0}, {4.8, 0, 0, 0}, {4, 0, 0, 0}, {3.2, 0, 0, 0}, {2.4, 0, 0, 0}, {
                                           1.6, 0, 0, 0},
                                       {0.8, 0, 0, 0}, {0, 0, 0, 0}};
  int i = 0;
  while (i < 43)
  {
    moveToPose(command_list_search[i][0], command_list_search[i][1], command_list_search[i][2],
               command_list_search[i][3]);
    i++;
  }
  while (home) ///sy wait till drone is outside of home
    ros::spinOnce();
  while (!detected_flag && !home)  ///sy wait till tag is detected or come back to home
  {
    ros::spinOnce();
  }

  if (detected_flag == true && home == false)
  {
    stopCmdAndHover();
    ndPause.sleep();
    ndPause.sleep();
    ndPause.sleep();

    //TODO put small range search in as part of centering tag

    if (centeringTag(DESIRED_HEIGHT))
    {
      picture_flag = true;
      ros::spinOnce();
    }
    else
    {
      ROS_INFO("Centering tag outside home area");
      centeringTag(DESIRED_HEIGHT);
    }
  }
  else
  {
    //search finished but failed to find tag
    ROS_INFO("Search complete but tag not found");
  }
}

/** Centering the target tag when the target tag is detected.
 *
 */
bool PRGPARDrone::centeringTag(double current_height)
{

  //Update tag information
  ros::spinOnce();
  if (smallRangeSearch())  ///sy even if the tag is in sight?
  //if (detected_flag == true)
  {
//    while ((tag_x_coord < 400 || tag_x_coord > 600 || tag_y_coord < 400 || tag_y_coord > 600 || (home && tag_orient > 183)
//        || (home && tag_orient < 177)) && detected_flag == true)
//    {
    if(detected_flag == true) ///sy if small range search fails, it is false obviously.
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
        ndPause.sleep();
        ros::spinOnce();
      }
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

/** Fly to the target when the target tag is not detected.
 *  Firstly, initialise the AR.Drone and then send the commands to control the flight.
 */
void PRGPARDrone::flightToTarget()
{
//if (initialising_PTAM_flag == true)
//{
//initARDrone();
//}
//else if (aligning_to_home_tag == true)
//{
//centeringTag();
//}
//else if (detected_flag == false)
//{
//flightToSearchTag();
//}

}

/** Send commands to fly home.
 *
 */
void PRGPARDrone::flightToHome()
{
  ros::spinOnce();
  ROS_INFO("Current position is : %0.2f, %0.2f", currentPos_x, currentPos_y);

}

/** The mai n running loop for the prgp_ardrone package.
 *  Getting the command from Pi-Swarm to start the AR.Drone. Then flight to the target. centering
 *  the target, taking the picture, returning home and send command to return the Pi-Swarm. All the
 *  functions are organised by the flags (true and false).
 */
void PRGPARDrone::run()
{

  ROS_INFO("Starting running");

  if (ros::ok())
  {
    if(initARDrone())
    {
      while(1)
      {
        ros::spinOnce();
      }
      //search fly path function
      flightToSearchTag();

      flightToHome();

      ndPause.sleep();
      ndPause.sleep();
      ndPause.sleep();
      ndPause.sleep();
      ndPause.sleep();
      sendFlightCmd("c land");
      ndPause.sleep();
      ndPause.sleep();

      ndPause.sleep();
      ndPause.sleep();
      ndPause.sleep();
      ndPause.sleep();
      //      std::cout << "**** executing_command_flag" << executing_command_flag << " after command sent" << std::endl;
      //      ros::spinOnce();
      //      std::cout << "**** executing_command_flag" << executing_command_flag << " after spinOnce" << std::endl;
      //      executing_command_flag = true;
      //      std::cout << "**** executing_command_flag" << executing_command_flag << " after reset" << std::endl;
    }
    else
    {
      ROS_INFO("Drone initialisation failed");
      sendFlightCmd("c land");
    }
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

#else

void piswarmCmdRev(const std_msgs::StringConstPtr str)
{
  //ROS_INFO("%s",str->data.substr(0,1));
  ROS_INFO_STREAM(*str);
  ROS_INFO("%s\n",str->data.c_str());
  //ROS_INFO("%s\n",str->data.substr(0,2));
  std::string k = str->data.substr(0,1);
  ROS_INFO("%s\n",k.c_str());
}

void takePic(const sensor_msgs::ImageConstPtr img)
{
//store;
//show;
}

void tagResult(const ardrone_autonomy::Navdata &navdataReceived)
{
  if(navdataReceived.tags_count > 0)
  {
    //Send confirmation to piswarm, use publish
    tag_detected = true;
  }
}
int * a;
int x = 5;
a = &x;
print (x) -> 5
print(a) -> 8767856
print(&x) -> 8767856
print(*a) -> 5
/**
 *  callback function
 */
void currentPos(const tum_ardrone::filter_state& currentPos)
{
  currentPos_x = currentPos.x;
  currentPos_y = currentPos.y;
}

/**
 *  main function starts here.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "prgp_ardrone"); //Create node
  ros::NodeHandle ndh_;
  ros::Duration ndPause;

  //Publishers
  ros::Publisher landPub;//send landing commands
  ros::Publisher takeoffPub;//send takeoff commands
  ros::Publisher drone_pub;//send commands to AR.Drone
  ros::Publisher cmdPub;//To sen cnd to PiSwarm
  ros::Publisher velPub;//send cmd directly to cmd_vel topic of the ardrone_autonomy

  //Subscribers
  ros::Subscriber cmdSub;//To get cnd from PiSwarm
  ros::Subscriber tagSub;//To get Tag detection result
  ros::Subscriber currentPosSub;
  ros::Subscriber imgSub;

  ros::ServiceClient toggleCamSrv;
  ros::ServiceClient detecttypeSrv;

  ndPause = ros::Duration(2,0);
  /*Publishers*/
  //if sometimes the topic cannot be reslived, try to change the topic below to "ndh_.resolveName("topic")"
  landPub = ndh_.advertise<std_msgs::Empty>("/ardrone/land",1);
  takeoffPub = ndh_.advertise<std_msgs::Empty>("/ardrone/takeoff",1);
  drone_pub = ndh_.advertise<std_msgs::String>("tum_ardrone/com",50);
  cmdPub = ndh_.advertise<std_msgs::String>("piswarm_com", 1);
  velPub = ndh_.advertise<geometry_msgs::Twist>("cmd_vel",1);

  /*Subscribers*/
  //if sometimes the topic cannot be reslived, try to change the topic below to "ndh_.resolveName("topic")"
  cmdSub = ndh_.subscribe("piswarm_com", 1, piswarmCmdRev);
  tagSub = ndh_.subscribe("/ardrone/navdata", 1, tagResult);
  currentPosSub = ndh_.subscribe("/ardrone/predictedPose", 1, currentPos);
  imgSub = ndh_.subscribe("ardrone/image_raw",10, takePic);

  toggleCamSrv = ndh_.serviceClient<std_srvs::Empty>("ardrone/togglecam",1);
  detecttypeSrv = ndh_.serviceClient<std_srvs::Empty>("ardrone/detecttype",1);

  ndPause.sleep();//Wait for 2 seconds to prepare publishers & subscribers
  //int i = 1;

  while(ros::ok())
  {
    ROS_INFO("Hi from Liu");
    ndPause.sleep();
    /* if(i == 1)
     {
     ROS_INFO("change the detect type");
     detecttypeSrv.call(detect_srvs);
     i =0;
     }*/

    /* if(i == 1)
     {
     ROS_INFO("toggle the camera");
     toggleCamSrv.call(toggle_srvs);
     i =0;
     }*/

    /*
     c_Pi = " ";
     c_Pi = "b";
     s_Pi.data = c_Pi.c_str();
     cmdPub.publish(s_Pi);
     ndPause.sleep();
     ndPause.sleep();
     */

    ros::spinOnce();
  }
  return 0;
}
#endif //end of CLASS_STYLE

