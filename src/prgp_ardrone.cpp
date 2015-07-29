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
 *  @brief The source file to program and test the ardrone.
 *  @details This is initially created and proposed by Chengqing Liu
 *  @version 0.9
 *  @author
 *  @date 24 July 2015
 *  @copyright BSD License.
 */

#include <prgp_ardrone/prgp_ardrone.h>


/**
 *  callback function
 */
void piswarmCmdRev(const std_msgs::StringConstPtr str)
{
	//ROS_INFO("%s",str.data);
}

/**
 *  callback function
 */
void takePic(const sensor_msgs::ImageConstPtr img)
{
  //store;
  //show;
}

/**
 *  callback function
 */
void tagResult(const ardrone_autonomy::Navdata &navdataReceived)
{
  if(navdataReceived.tags_count > 0)
  {
   //Send confirmation to piswarm, use publish
    tag_detected = true ;
  }
}

/**
 *  callback function
 */
void currentPos(const tum_ardrone::filter_state& currentPos)
{
  currentPos_x = currentPos.x;
  currentPos_y = currentPos.y;
}

/*
void sendCmdToPiswarm()
{
  c_Pi = " ";
  c_Pi = "b";
  s_Pi.data = c_Pi.c_str();
  cmdPub.publish(s_Pi);
}

void sendVelCmd()
{
	cmdT.angular.z =0; // -cmd.yaw;
	cmdT.linear.z = 0; //cmd.gaz;
	cmdT.linear.x = 0; //-cmd.pitch;
	cmdT.linear.y = 0; //-cmd.roll;
	cmdT.angular.x = cmdT.angular.y = 0; //gui->useHovering ? 0 : 1;
	vel_pub.publish(cmdT);
}

void takeOff()
{
  ROS_INFO("Arrive home");
  c = " ";
  takeoffPub.publish(std_msgs::Empty());
  ROS_INFO("Takeoff");

}

void land()
{
  ROS_INFO("Arrive home");
  c = " ";
  landPub.publish(std_msgs::Empty());
  ROS_INFO("Land");

}

void sendFlightCnd()
{
  s.data = c.c_str();
  pthread_mutex_lock(&send_CS);
  drone_pub.publish(s);
  pthread_mutex_unlock(&send_CS);
}

void toggleCam()
{
  pthread_mutex_lock(&send_CS);
  toggleCamSrv.call(toggle_srvs);
  pthread_mutex_unlock(&send_CS);

  //current_cam = (current_cam + 1) % 2;
  //ARDRONE_TOOL_CONFIGURATION_ADDEVENT(video_channel, &current_cam, NULL);
  //ROS_INFO("Setting camera channel to vertical");
}

void changeTargetTag()
{
  uint16_t new_tag = TAG_TYPE_MASK(TAG_TYPE_ORIENTED_ROUNDEL);
  switch(target_tag)
  {
    case 0:

      //ARDRONE_TOOL_CONFIGURATION_ADDEVENT(detections_select_v, &new_tag, NULL);
    case 1:

    case 2:

    default:
    break;
  }
}

void initialArdrone()
{
  c = " ";
  c = "c autoTakeOver 500 800";
  sendFlightCnd();

  ROS_INFO("Current pos: X = [%f] Y = [%f]", currentPos_x, currentPos_y);
  c = " ";
  c = "c setMaxControl 0.1";
  sendFlightCnd(); //set AR.Drone speed limit

  c = " ";
  c = "c setInitialReachDist 0.25";
  sendFlightCnd();

  c = " ";
  c = "c setStayWithinDist 0.25";
  sendFlightCnd();

  c = " ";
  c = "c setStayTime 3";// stay 3 seconds
  sendFlightCnd();

  c = " ";
  c = "c lockScaleFP";//PTAM
  sendFlightCnd();

  ndPause.sleep();
}


uint16_t flightToTarget()
{
  // Set current position to be the origin
  c = "c setReference $POSE$";
  sendFlightCnd();
  ROS_INFO("Reference is set");
  ndPause.sleep();

  // Takeoff drone
  takeOff();
  ndPause.sleep();
  ndPause.sleep();
  ndPause.sleep();

  // Set initial parameters
  initialArdrone();

  c = " ";//clear the command, avoid the overlap
  //sprintf(&c[0],"c goto %.2f %.2f 1.0 %.2f");
  sendFlightCnd();
  //ROS_INFO("fly to the point: x = [%f], y = [%f]");

  changeTargetTag();//change the target tag depending on the piswarm repor.

  //Search tag in path_plan, if not found, keep searching or come back.
  if(tag_detected == false)
  {
    c = " ";
    //sprintf(&c[0],"c goto %.2f %.2f 1.0 %.2f");
    sendFlightCnd();
    //ROS_INFO("fly to the point: x = [%f], y = [%f]");
  }
  else
  {
    toggleCam();
    takePic();
    sendCmdToPiswarm();
    return COME_BACK;
  }
}

uint16_t flightToHome()
{
  //Clear commands
  c = " ";
  c = "c clearCommands";
  sendFlightCnd();

  //Return to point out from the gantry
  //Need to design and set the point position
  c = " ";
  //sprintf(&c[0],"c goto %.2f %.2f 1.0 %.2f");
  sendFlightCnd();

  //Return home
  c = " ";
  c = "c goto -0.25 -0.25 0.25 0";
  sendFlightCnd();
} detecttypeSrv.call(detect_srvs);
*/

/**
 *  main function starts here.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "prgp_ardrone");	//Create node
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

  ndPause.sleep(); //Wait for 2 seconds to prepare publishers & subscribers
  //int i = 1;


  while(ros::ok())
  {
	  ROS_INFO("Hi from Liu");
	  ndPause.sleep();
	  ndPause.sleep();
	 /* if(i == 1)
	  {
	    ROS_INFO("change the detect type");
	    detecttypeSrv.call(detect_srvs);
	    i =0;
	  }*/
	  ndPause.sleep();
	  ndPause.sleep();
	  //new_tag = TAG_TYPE_MASK(TAG_TYPE_BLACK_ROUNDEL);
	  //ARDRONE_TOOL_CONFIGURATION_ADDEVENT(detections_select_v, &new_tag, NULL);
	  //ardrone_application_default_config.detections_select_v = TAG_TYPE_MASK(TAG_TYPE_BLACK_ROUNDEL);
   /*
    if(cnd_rev == START)
    {

      flight_task_completed =flightToTarget();
    }
    if(flight_task_completed == COME_BACK)
    {
      flightToHome();
    }
    */
    ros::spinOnce();
  }
  return 0;
}




