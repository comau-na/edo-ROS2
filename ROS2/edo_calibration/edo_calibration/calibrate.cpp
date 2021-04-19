#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

static const std::string OPENCV_WINDOW = "Image window";
using std::placeholders::_1;

cv::Mat src, src_gray;
cv::Mat dst, detected_edges;
int lowThreshold = 0;
const int max_lowThreshold = 100;
const int ratio = 3;
const int kernel_size = 3;

static void CannyThreshold(int, void*)
{
    cv::blur( src_gray, detected_edges, cv::Size(3,3) );
    cv::Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );
    dst = cv::Scalar::all(0);
    src.copyTo( dst, detected_edges);
    imshow( OPENCV_WINDOW, dst );
}

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
    using namespace cv;

    cv_bridge::CvImagePtr cv_ptr;
    rclcpp::TimerBase::SharedPtr timer_;

    try
    {
      // Subscribe to input video feed and publish output video feed
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      src = cv_ptr->image;
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    dst.create( src.size(), src.type() );
    cvtColor( src, src_gray, cv::COLOR_BGR2GRAY );
    namedWindow( OPENCV_WINDOW, cv::WINDOW_AUTOSIZE );
    cv::createTrackbar( "Min Threshold:", OPENCV_WINDOW, &lowThreshold, max_lowThreshold, CannyThreshold );
    CannyThreshold(0, 0);
    
    // Update GUI Window
    //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    
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
