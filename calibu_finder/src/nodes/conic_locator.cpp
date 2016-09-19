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

//Message defs
#include <calibu_finder/conic_locator.h> 
#include <calibu_finder/conicData.h>


using std::string;
using std::cout;
using std::endl;

using boost::shared_ptr;
using boost::make_shared;

using calibu_finder::conic_locator;
using calibu_finder::conicData;

class ConicLocatorService 
{
public:
  ConicLocatorService(ros::NodeHandle nh);
  ~ConicLocatorService()  {  } ;
  bool executeCallBack( conic_locator::Request &req, conic_locator::Response &res);


private:
  ros::NodeHandle nh_;
  ros::ServiceServer conic_locate_server_;

};

ConicLocatorService::ConicLocatorService(ros::NodeHandle nh)
{
  
  nh_ = nh;
  ros::NodeHandle pnh("~");
  // camera parameters come from the request

  conic_locate_server_ = nh_.advertiseService( "ConicLocateService", &ConicLocatorService::executeCallBack, this);
  ROS_INFO("Conic locator service started");
}

bool ConicLocatorService::executeCallBack( conic_locator::Request &req, conic_locator::Response &res)
{
  ros::NodeHandle nh;

  //Suck an image out of the pipe:
  ROS_INFO("Reading image from [%s]", req.image_topic.c_str());

  std::string target_type_ = req.target_type; 
  ROS_INFO("Using target type [%s]", req.target_type.c_str());
  calibu::TargetGridDot target( target_type_); // load a target dot grid from a preset

 
  sensor_msgs::ImageConstPtr cal_image = ros::topic::waitForMessage<sensor_msgs::Image>(req.image_topic);

  //The image has to be gray8 to work with Calibu's grid finder, see Calibu/src/image/ImageProcessing.cpp

  if (cal_image->encoding != sensor_msgs::image_encodings::MONO8)
    {
      ROS_FATAL("Bad image format [%s], needs to be mono8", cal_image->encoding.c_str());
      return false;
    }
  
  calibu::Tracker tracker( target, cal_image->width, cal_image->height );

  if( !tracker.ProcessFrame((const unsigned char*)&cal_image->data[0], cal_image->width, cal_image->height,  cal_image->step))
    {
      ROS_INFO("Failed to track!\n");
      return false;
    }


  //The result is a list of conics for this frame
  const std::vector<int>& conics_target_map = tracker.ConicsTargetMap();

 
  for (size_t i = 0; i < tracker.GetConicFinder().Conics().size(); ++i)
      {
	const Eigen::Vector3d& pos_3d = target.Circles3D()[conics_target_map[i]];
	if(conics_target_map[i] < 0 )
	  {
	    continue;
	  }

	conicData theConic;
	theConic.id = conics_target_map[i];
	theConic.u = tracker.GetConicFinder().Conics()[i].center[0];
	theConic.v = tracker.GetConicFinder().Conics()[i].center[1];
	theConic.x = pos_3d[0];
	theConic.y = pos_3d[1];
	theConic.z = pos_3d[2];
	res.conics.push_back(theConic);

      }
    return true;
   
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "conic_locator_service");
  ros::NodeHandle node_handle;
  ConicLocatorService conic_locator(node_handle);
  ros::spin();
  ros::waitForShutdown();
  return 0;
}
