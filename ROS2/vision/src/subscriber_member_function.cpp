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
#include <string>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "common.hpp"
#include <functional>
#include <memory>
#include <blocks_buckets.h>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"


using std::placeholders::_1;
//Subscriber nodes
class EdoVision : public rclcpp::Node
{
public:
  EdoObjDetection()
  : Node("edo_ObjectDetection")
  {
    subscription_obj = this->create_subscription<vision_msgs::msg::Detection2DArray>(
      "/detection", 10, std::bind(&EdoVision::detectionCallback, this, _1));
  }

  EdoClassification()
  : Node("edo_Classification")
  {
     subscription_class = this->create_subscription<vision_msgs::msg::Classification2D>(
      "/detection", 10, std::bind(&EdoVision::classificationCallback, this, _1));
  }

private:
  void detectionCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg) 
  {
    std::cout << "detection msg contents start: " << std::endl;
    for(vision_msgs::msg::Detection2D detection : msg->detections){
      // just for testing. print out values
      std::cout << "detection msg contents end: " << std::endl;
    }    

  }
   void classificationCallback(const vision_msgs::msg::Classification2D::SharedPtr msg) 
  {
    std::cout << "classifiation msg contents start: " << std::endl;
    for(vision_msgs::msg::Classification2D classification : msg->results){
      // just for testing. print out values
      std::cout << "classifiation msg contents start:" << std::endl;
};

//class1 or 2. 1 = Bucket, 2 = Block, 3 = Unknown?
class Blocks
{
  int classification;
  double block_curr_position;
  std::string blockColor;
  
   Block( int classification, double curr_position, std::string blockColor)
  {
    this->classification = classification;
    this->block_curr_position = curr_position;
    this->std::string blockColor;
  }

  boolean isBlock(int classification)
  {
    if(classification == 2)
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  double getBlockPosition()
  {
    return curr_position;
  }
  
  void setBlockPostion( int p)
  {
    p = curr_position;
  }

  std::string getBlockColor()
  {
    return blockColor;
  }

  setBlockColor(std::string c)
  {
    c = blockColor;
  }

};

class Buckets
{
  int classification;
  double bucket_curr_position;
  std::string blockColor;  


  Bucket( int classification, double curr_position, std::string bucketColor)
  {
    this->classification = classification;
    this->curr_position = curr_position;
    this->std::string blockColor; 
  }

 boolean isBucket( int classification)
 {
   if(classification == 1)
   {
     return true;
   }
   else 
   {
     return false;
   }
 }

  double getBucketPosition()
  {
    return curr_position;
  }

  setBucketPosition(double pos)
  {
    pos = curr_position;
  }

 std::string getBucketColor()
  {
    return bucketColor;
  }

  setBucketColor(std::string col)
  {
    col = bucketColor;
  }
};



class EdoSorting
{
public:

   //to save Buckets
  std::vector<Blocks> BucketVector;
  //to save Blocks
  std::vector<Buckets> BlockVector;

  void readEnvironment(int classification)
  
  {    
    //if the camera has detected something// cant use this in if
    if(vision_msgs::msg::Detection2D detection : msg->detections =! NULL)
    {
      //loop thru detection array
      for (i = 0, i<vision_msgs::msg::Detection2D detection : msg->detections; i++)
      {
        //check if Block or bucket
        if(isBlock()==true)
        {
          //if is block, create block obj
          //get block location
          //get block classification
          Blocks block;
          block.setBlockPositon(i);
          for(j = 0; j < vision_msgs::msg::Classification2D classification : msg->results)
          {
            block.setBlockColor
          }
          else{
             //if(detection == bucket)
            // create bucket obj
            //get bucket location
            //get bucket classfication
            Buckets bucket;
            bucket.setBucketPositon(i);
            for(j = 0; j < vision_msgs::msg::Classification2D classification : msg->results)
              {
                bucket.setBucketColor
              }

          }
        }
      
      }
     

    }
  };


  void ExecuteCommand(std::vector BucketVector, std::vector BlockVector)
  {
     //take in all buckets, blocks from readEnvironment
   
     //recieve block current postion, bucket postion is declared as destination coordinate(center of bucket) if block is not inside bucket
      //matches block and bucket classes
      //send messege to movement wrapper to navigate block to bucket destination
      //gripper logic
 
  }


};




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
};