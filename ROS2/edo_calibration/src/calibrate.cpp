#include <ros/ros.h>
#include "cv_bridge/cv_bridge.h"
#include "opencv2/core/core.hpp"
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter: public rclcpp::Node
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;

public:
  ImageConverter()
    : Node("opencvJunk"), count_(0)
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));


    // Subscrive to input video feed and publish output video feed
    CvImageConstPtr toCvShare(const sensor_msgs::ImageConstPtr& source,
                const std::string& encoding = std::string());
    
    

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
