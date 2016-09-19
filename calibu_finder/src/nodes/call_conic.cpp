/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2014, Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.


 * Modified to add the image topic for the calibrator to use, to support multiple cals at once
 * S. McGuire

 */

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <calibu_finder/conic_locator.h>
#include <calibu_finder/conicData.h> 
#include <tf/transform_broadcaster.h>

using std::string;
using std::vector;

class callService
{
public:
  callService(ros::NodeHandle nh): nh_(nh)
  {

    ros::NodeHandle pnh("~") ;

    if(!pnh.getParam("optical_frame", optical_frame_)){
      optical_frame_ = "basler1_optical_frame";
    }

    if(!pnh.getParam("image_topic", image_topic_)){
      image_topic_ = "/image_raw";
    }

    if(!pnh.getParam("target_type", target_type_)){
      target_type_ = "letter";
    }

    if(!pnh.getParam("conic_locate_service", conic_service_name_)){
      conic_service_name_ = "ConicLocateService";
    }
    
    client_ = nh_.serviceClient<calibu_finder::conic_locator>(conic_service_name_);

    setRequest();

  }
  bool callTheService();
  void copyResponseToRequest();
  void setRequest();

private:
  ros::NodeHandle nh_;
  ros::ServiceClient client_;
  calibu_finder::conic_locator srv_;


  std::string optical_frame_;
  std::string target_frame_;
  std::string conic_service_name_;
  std::string image_topic_;
  std::string target_type_;
};

void callService::copyResponseToRequest()
{
  setRequest();

}
bool callService::callTheService()
{
  if(client_.call(srv_)){
    //Publish the conic list as a Measurement
    ROS_INFO("Conic count: %lu",
	     srv_.response.conics.size());
    uint32_t conicCount = srv_.response.conics.size();
    for (int i=0; i< conicCount; i++)
      {
	calibu_finder::conicData theConic = srv_.response.conics[i];
	ROS_INFO("Conic: %u, u: %1.2f v: %1.2f x: %1.2f y:%1.2f z:%1.2f",
		 theConic.id,
		 theConic.u,
		 theConic.v,
		 theConic.x,
		 theConic.y,
		 theConic.z);
      }
    return(true);
  }
  ROS_ERROR("Conic Location Failure");
  return(false);
}

void callService::setRequest()
{
    srv_.request.image_topic = image_topic_;
    srv_.request.target_type = target_type_;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "call_conic");
  ros::NodeHandle nh;
  callService call_service(nh);
  ros::Rate loop_rate(10);
  while(ros::ok()){
    if(call_service.callTheService()){
      call_service.copyResponseToRequest();
    }
    else{
      call_service.setRequest();
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}
