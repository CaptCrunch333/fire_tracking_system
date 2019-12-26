#pragma once

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Float32.h>
#include "ROSUnit.hpp"
#include "ThermalImageMsg.hpp"

class ImageConverter : public ROSUnit
{

public:

	ImageConverter(std::string, ros::NodeHandle&);
	~ImageConverter();
	//passes the input topic
	void getImageTopic(std::string& tmp);
	//change the input topic
	void setImageTopic(std::string tmp);
	//passes the output topic
	void showImage();
	void receive_msg_data(DataMessage* t_msg) {};

private:

	std::string CV_WINDOW;
	std::string INPUT_TOPIC;
    std::string OUTPUT_TOPIC;
	ThermalImageMsg m_image;

	//ROS
	cv_bridge::CvImagePtr cv_ptr;
	ros::Publisher chatter_pub;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;


	// ros <-> OpenCV
	void ros2cv(const sensor_msgs::ImageConstPtr& in_msg);
	bool getImage(cv_bridge::CvImagePtr& tmp);
};
