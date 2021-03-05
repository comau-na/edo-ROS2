#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/core/core.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";
using std::placeholders::_1;

class ImageConverter: public rclcpp::Node
{
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

public:
  ImageConverter()
    : Node("opencvJunk")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/edo/camera/image_raw", 10, std::bind(&ImageConverter::imageCb, this, _1));

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::msg::Image::SharedPtr msg) const
  {
    cv_bridge::CvImagePtr cv_ptr;
    rclcpp::TimerBase::SharedPtr timer_;

    try
    {
      // Subscribe to input video feed and publish output video feed
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  ImageConverter ic;
  rclcpp::spin(std::make_shared<ImageConverter>());
  return 0;
}
