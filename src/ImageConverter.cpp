#include "ImageConverter.hpp"

//#include <ros/ros.h>
ImageConverter::ImageConverter(std::string tmp, ros::NodeHandle& _nh) : it_(_nh), ROSUnit(_nh)
{
    CV_WINDOW = "CV Image ";
    INPUT_TOPIC = tmp;
    OUTPUT_TOPIC = INPUT_TOPIC + "_output";
	image_sub_ = it_.subscribe(INPUT_TOPIC, 1, &ImageConverter::ros2cv, this);
    image_pub_ = it_.advertise(OUTPUT_TOPIC, 1);
    cv::namedWindow(CV_WINDOW, cv::WINDOW_NORMAL);
}

ImageConverter::~ImageConverter()
{
    cv::destroyWindow(CV_WINDOW);
}

void ImageConverter::getImageTopic(std::string& tmp)
{
    tmp = INPUT_TOPIC;
}

void ImageConverter::setImageTopic(std::string tmp)
{
    INPUT_TOPIC = tmp;
   image_sub_ = it_.subscribe(INPUT_TOPIC, 1, &ImageConverter::ros2cv, this);

}

void ImageConverter::ros2cv(const sensor_msgs::ImageConstPtr& in_msg)
{
    cv_bridge::CvImagePtr tmp_image;
	try
    {
      cv_ptr = cv_bridge::toCvCopy(in_msg, sensor_msgs::image_encodings::BGR8);
      if(getImage(tmp_image))
      {
        m_image.image = tmp_image->image;
        showImage();
        this->emit_message((DataMessage*) &m_image);
      }
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
}

bool ImageConverter::getImage(cv_bridge::CvImagePtr& tmp)
{
    if(cv_ptr != NULL)
    {
        tmp = cv_ptr;
        return true;
    }
    else
    {
        return false;
	}
}

void ImageConverter::showImage()
{
    // Update GUI Window
    cv::imshow(CV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
}

