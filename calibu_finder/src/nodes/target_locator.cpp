/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2014, Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>

#include <calibu/target/TargetGridDot.h>
#include <calibu/pose/Tracker.h>
#include <calibu/pose/Pnp.h>
#include <calibu/cam/camera_xml.h>

#include <calibu_finder/target_locator.h> 

using std::string;
using std::cout;
using std::endl;

using boost::shared_ptr;
using boost::make_shared;

using calibu_finder::target_locator;

class TargetLocatorService 
{
public:
  TargetLocatorService(ros::NodeHandle nh);
  ~TargetLocatorService()  {  } ;
  bool executeCallBack( target_locator::Request &req, target_locator::Response &res);


private:
  ros::NodeHandle nh_;
  ros::ServiceServer target_locate_server_;



};

TargetLocatorService::TargetLocatorService(ros::NodeHandle nh)
{
  
  nh_ = nh;
  ros::NodeHandle pnh("~");


  // camera parameters come from the request
  target_locate_server_ = nh_.advertiseService( "TargetLocateService", &TargetLocatorService::executeCallBack, this);
}

bool TargetLocatorService::executeCallBack( target_locator::Request &req, target_locator::Response &res)
{
  ros::NodeHandle nh;

  //Suck an image out of the pipe:
  ROS_INFO("Reading image from [%s]", req.image_topic.c_str());

  std::string target_type_ = req.target_type; 
  ROS_INFO("Using target type [%s]", req.target_type.c_str());
  calibu::TargetGridDot target( target_type_); // load a target dot grid from a preset

  //Instantiate a camera model from the request:

  //Rehydrate the model
  std::shared_ptr<calibu::Rigd> rig = calibu::ReadXmlRigFromString( req.camera_xml);
  std::shared_ptr<calibu::CameraInterfaced> model = rig->cameras_[0];
  ROS_INFO("Using rig with [%lu] cameras", rig->cameras_.size());
      
  sensor_msgs::ImageConstPtr cal_image = ros::topic::waitForMessage<sensor_msgs::Image>(req.image_topic);

  //The image has to be gray8 to work with Calibu's grid finder, see Calibu/src/image/ImageProcessing.cpp

  if (cal_image->encoding != sensor_msgs::image_encodings::MONO8)
    {
      ROS_FATAL("Bad image format [%s], needs to be mono8", cal_image->encoding.c_str());
      return false;
    }
  
  calibu::Tracker tracker( target, cal_image->width, cal_image->height );

  if( !tracker.ProcessFrame( model, (const unsigned char*)&cal_image->data[0], cal_image->width, cal_image->height,  cal_image->step))
    {
      ROS_INFO("Failed to track!\n");
      return false;
    }


    //Output is available at tracker.PoseT_gw()
    //cout << "Track:" << tracker.PoseT_gw().matrix() << endl;

    //Get the quaternion
    const Eigen::Quaternion<double> quat =  tracker.PoseT_gw().unit_quaternion();

    res.final_pose.orientation.x = quat.x();
    res.final_pose.orientation.y = quat.y();
    res.final_pose.orientation.z = quat.z();
    res.final_pose.orientation.w = quat.w();
    
    const Eigen::Matrix<double, 3, 1> trans =  tracker.PoseT_gw().translation();
    res.final_pose.position.x = trans(0,0);
    res.final_pose.position.y = trans(1,0);
    res.final_pose.position.z = trans(2,0);
    
    res.final_cost_per_observation = 0.0; //requires rms as a class variable in Tracker.cpp
    
    return true;
   
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "target_locator_service");
  ros::NodeHandle node_handle;
  TargetLocatorService target_locator(node_handle);
  ros::spin();
  ros::waitForShutdown();
  return 0;
}
