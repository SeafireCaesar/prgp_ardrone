#include "prgp_ardrone/image.h"
pthread_mutex_t DroneImage::image_mt = PTHREAD_MUTEX_INITIALIZER;
DroneImage::DroneImage(const sensor_msgs::ImageConstPtr img, const std::string& encoding = std::string())
{
  cv_ptr = cv_bridge::toCvCopy(img, encoding);
  pthread_mutex_lock(&image_mt);
  width = img->width;
  height = img->height;
  pthread_mutex_unlock(&image_mt);
  if (new_image.size().x != width || new_image.size().y != height)
    new_image.resize(CVD::ImageRef(width, height));
  stage = 1;
}
DroneImage::~DroneImage(void)
{
  if (window != NULL)
    delete (window);
}
bool DroneImage::update(void)
{
  switch (stage)
  {
    case 0:
      break;

    case 1: ///sy copying the data
      std::cout << "stage 1" << std::endl;
      pthread_mutex_lock(&image_mt);
      memcpy(new_image.data(), cv_ptr->image.data, width * height * 3); ///sy cpy the image to mimFrameBW.data()
      pthread_mutex_unlock(&image_mt);
      stage = 2;
      return false;
      break;
    case 2: ///sy showing the window
      std::cout << "stage 2" << std::endl;
      if (window != NULL)
        delete (window);
      window = new CVD::VideoDisplay(new_image.size());
      glDrawPixels(new_image);
      glFlush();
      stage = 3;
      break;
    case 3: ///sy saving data
      std::cout << "stage 3" << std::endl;
      image_file.open("output.bmp", std::fstream::out);
      CVD::img_save(new_image, image_file, CVD::ImageType::BMP);
      image_file.close();
      return true;
      break;

    default:
      return false;
      break;
  }
}
