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
#include <functional>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
 #include "vision_msgs/msg/detection2_d_array.hpp"
 #include "geometry_msgs/msg/pose2_d.hpp"
 #include "vision_msgs/msg/detection2_d.hpp"


using std::placeholders::_1;
//Subscriber nodes
class EdoVision : public rclcpp::Node
{
  public:
    EdoVision()
    : Node("EdoVision")
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
 
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr subscription_;
};

class OBJ_Location : public rclcpp::Node
{
  public:
    OBJ_Location()
    : Node("OBJ_Location")
    {
      subscription_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
      "/detection", 10, std::bind(&OBJ_Location::OBJ_loc_Callback, this, _1));
    }
  private:
    void OBJ_loc_Callback(const geometry_msgs::msg::Pose2D::ConstPtr msg)
  {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->x.c_str());

  }
 
   rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr subscription_;
};


//class1 or 2. 1 = Bucket, 2 = Block, 3 = Unknown?
class Block
{
  public:

  std::string classification;
  float block_center_x;
  float block_center_y;
 
    Block( std::string classification, float block_center_x, float block_center_y)
  {
    this->classification = classification;
    this->block_center_x = block_center_x;
    this->block_center_y = block_center_y;
  }

  std::string getClassification(Block block)
  {
    return classification;
  }

   bool isBlock(vision_msgs::msg::Detection2D msg)
  {
    std::string result;
    msg.tracking_id = result;
    if (result == "cube" )
    {
      return true;
    }
    else
    {
    return false;
  }

  void setBlockValues(geometry_msgs::msg::Pose2D msg, Block block)
  {
    msg.x = block.block_center_x;
    msg.y = block.block_center_y;
  }

   void setBlockClassification(vision_msgs::msg::Detection2D msg, Block block)
  {
    msg.tracking_id = block.classification;
  }

};

class Bucket
{
  public:

  std::string classification;
  float bucket_center_x;
  float bucket_center_y;
 
    Bucket( std::string classification, float bucket_center_x, float bucket_center_y)
  {
    this->classification = classification;
    this->bucket_center_x = bucket_center_x;
    this->bucket_center_y = bucket_center_y;
  }

  std::string getClassification(Bucket bucket)
  {
    return classification;
  }

   bool isBucket(vision_msgs::msg::Detection2D msg)
  {
    std::string result;
    msg.tracking_id = result;
    if (result == "bucket" )
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  
  void setBucketValues(geometry_msgs::msg::Pose2D msg, Bucket bucket)
  {
    msg.x = bucket.bucket_center_x;
    msg.y = bucket.bucket_center_y;
  }

  void setBucketClassification(vision_msgs::msg::Detection2D msg, Bucket bucket)
  {
    msg.tracking_id = bucket.classification;
  }


};


void readEnvironment()
{
 //check if the camera has detected something
 
      //if so, loop thru detection array
      int i = 0;
for(i = 0; i < vision_msgs::msg::Detection2DArray msg.detections; i++)
    {
        //if is block, create block obj
        if(isBlock() == true)
        {
           Block newblock;
            newblock.setBlockValues();
            newblock.setBlockClassification();
        }

        if(isBucket() == true)
        {
         Bucket newbucket;
         newbucket.setBucketValues;
         newbucket.setBucketClassification;
        }
       
       //store blocks into block vector
       //store buckets into bucket vector
         
       
    }
}




//take in all buckets, blocks from readEnvironment
//recieve block current postion, bucket postion is declared as destination coordinate(center of bucket) if block is not inside bucket
//matches block and bucket classes
//send messege to movement wrapper to navigate block to bucket destination
//gripper logic

void ExecuteCommand()


int main(int argc, char * argv[])
{
 

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;
  //auto subscriber_node = std::make_shared<EdoVision>();

  rclcpp::spin(std::make_shared<EdoVision>());
  //exec.add_node(subscriber_node);
  //  rclcpp::spin(std::make_shared<OBJ_Location>());
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

