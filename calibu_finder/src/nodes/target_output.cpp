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
#include <fstream>

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

class TargetLocator
{
public:
  TargetLocator(ros::NodeHandle nh, std::string outputPath);
  ~TargetLocator();
  void executeCallback(const sensor_msgs::Image::ConstPtr& cal_image);
  std::shared_ptr<calibu::CameraInterfaced> model;
  calibu::TargetGridDot *target;
  std::string outputPath;
  FILE* outputFile;
private:
  ros::NodeHandle nh_;
  bool goodTrack;
  


};

TargetLocator::~TargetLocator()
  {
    if (outputFile) fclose(outputFile);
  }

TargetLocator::TargetLocator(ros::NodeHandle nh, std::string outputPath)
{
  
  nh_ = nh;
  ros::NodeHandle pnh("~");
  goodTrack = false;
  char fileName[64];
  sprintf(fileName, "%s/calibu_poses.csv", outputPath.c_str());
  outputFile = fopen(fileName, "w");
  
  // camera parameters come from the request
  
}

void TargetLocator::executeCallback(const sensor_msgs::Image::ConstPtr& cal_image)
{

  //The image has to be gray8 to work with Calibu's grid finder, see Calibu/src/image/ImageProcessing.cpp

  if ((cal_image->encoding != sensor_msgs::image_encodings::MONO8) && (cal_image->encoding != sensor_msgs::image_encodings::TYPE_8UC1))
    {
      ROS_FATAL("Bad image format [%s], needs to be mono8", cal_image->encoding.c_str());
      goodTrack = false;
      return;
    }
  
  calibu::Tracker tracker( *target, cal_image->width, cal_image->height );

  if( !tracker.ProcessFrame( model, (const unsigned char*)&cal_image->data[0], cal_image->width, cal_image->height,  cal_image->step))
    {
      if (goodTrack)
	ROS_INFO("Failed to track!\n");
      goodTrack = false;
      return;
    }

  goodTrack = true;
    //Output is available at tracker.PoseT_gw()
    //cout << "Track:" << tracker.PoseT_gw().matrix() << endl;

    //Get the quaternion & translation, output to file
  
    const Eigen::Quaternion<double> quat =  tracker.PoseT_gw().unit_quaternion();
    const Eigen::Matrix<double, 3, 1> trans =  tracker.PoseT_gw().translation();

    /* Write the output line */
    ros::Time curTime = cal_image->header.stamp;
    fprintf(outputFile, "%d.%09d, %0.12f, %0.12f,%0.12f", curTime.sec, curTime.nsec, trans(0,0),
	    trans(1,0), trans(2,0));
    fprintf(outputFile, ",%0.12f, %0.12f,%0.12f,%0.12f\n",
	    quat.x(),
	    quat.y(),
	    quat.z(),
	    quat.w());
    /*
    res.final_pose.position.x = trans(0,0);
    res.final_pose.position.y = trans(1,0);
    res.final_pose.position.z = trans(2,0);
    res.final_pose.orientation.x = quat.x();
    res.final_pose.orientation.y = quat.y();
    res.final_pose.orientation.z = quat.z();
    res.final_pose.orientation.w = quat.w();
    res.final_cost_per_observation = 0.0; //requires rms as a class variable in Tracker.cpp
    */
    
    return;
   
}
std::string loadCameraXML(std::string camera_model_file_)
{
  std::ifstream t(camera_model_file_);
  std::stringstream buffer;
  buffer << t.rdbuf();
  return buffer.str();
  
}

int main(int argc, char** argv)
{
  std::string image_topic_;
  std::string camera_model_file_;
  std::string camera_model_xml_;
  std::string target_type_;
  
  ros::init(argc, argv, "target_output", ros::init_options::AnonymousName );
  ros::NodeHandle pnh("~");
  ROS_INFO("Writing to dir: [%s]", getenv("PWD"));
  TargetLocator target_locator(pnh,getenv("PWD"));
   ROS_INFO("Instantiated locator");
  image_transport::ImageTransport *it = new image_transport::ImageTransport(pnh);
  image_transport::Subscriber subs;
  
  if(!pnh.getParam("image_topic", image_topic_)){
    image_topic_ = "/image_raw";
  }
  
  if(!pnh.getParam("model_file", camera_model_file_)){
    camera_model_file_ = "cameras.xml";
  }
  
  if(!pnh.getParam("target_type", target_type_)){
    target_type_ = "letter";
  }


  ROS_INFO("Reading images from [%s]", image_topic_.c_str());
  ROS_INFO("Using target type [%s]", target_type_.c_str());
  
  calibu::TargetGridDot target( target_type_); // load a target dot grid from a preset

  //Instantiate a camera model from the request:
  camera_model_xml_ = loadCameraXML(camera_model_file_);
  std::shared_ptr<calibu::Rigd> rig = calibu::ReadXmlRigFromString( camera_model_xml_);
  std::shared_ptr<calibu::CameraInterfaced> model = rig->cameras_[0];
  ROS_INFO("Using rig with [%lu] cameras", rig->cameras_.size());

  target_locator.model = model;
  target_locator.target = &target;
  
  //Subscribe to the image stream:
  subs = it->subscribe(image_topic_, 1000, &TargetLocator::executeCallback, &target_locator);
  while (ros::ok())
    {
      ros::spinOnce();
    }

  return 0;
}
