// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"


using std::placeholders::_1;

class EdoVision : public rclcpp::Node
{
public:
  EdoObjDetection()
  : Node("edo_ObjectDetection")
  {
    subscription_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
      "/detection", 10, std::bind(&EdoVision::detectionCallback, this, _1));
  }

  EdoClassification()
  : Node("edo_Classification")
  {
     subscription_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
      "/detection", 10, std::bind(&EdoVision::detectionCallback, this, _1));
  }

private:
  void detectionCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg) 
  {
    std::cout << "test" << std::endl;
    for(vision_msgs::msg::Detection2D detection : msg->detections){
      // just for testing. print out values
      std::cout << "test" << std::endl;
    }    

  }
}


class readEnviroment : public rclcpp::Node
{
  
}



int main(int argc, char * argv[])
{
  

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;
  //auto subscriber_node = std::make_shared<EdoVision>();

  rclcpp::spin(std::make_shared<EdoObjDetection>());
  //exec.add_node(subscriber_node);
   rclcpp::spin(std::make_shared<EdoClassification>());
  //exec.add_node(subscriber_node);

  std::chrono::nanoseconds timeout;
  timeout= std::chrono::nanoseconds { 100000000 };

  /*
  while(rclcpp::ok()){
    exec.spin_some();
    
    //std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
  }
  */
  rclcpp::shutdown();
  return 0;
}