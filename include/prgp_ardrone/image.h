#ifndef PRGP_ARDRONE_INCLUDE_IMAGE_H_
#define PRGP_ARDRONE_INCLUDE_IMAGE_H_

#include <cvd/image_io.h>
#include <cvd/videodisplay.h>
#include <cvd/gl_helpers.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <iostream>
#include <sensor_msgs/Image.h>
#include <pthread.h>

class DroneImage
{
private:
  std::fstream image_file;
  CVD::Image<CVD::Rgb<CVD::byte> > new_image;
  cv_bridge::CvImagePtr cv_ptr;
  static pthread_mutex_t image_mt;
  CVD::VideoDisplay * window = NULL;
  unsigned int stage;
  int width;
  int height;
public:
  DroneImage(const sensor_msgs::ImageConstPtr img, const std::string& encoding);
  ~DroneImage(void);
  bool update(void);

};
#endif /* PRGP_ARDRONE_INCLUDE_IMAGE_H_*/
