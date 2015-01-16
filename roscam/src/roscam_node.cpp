// Sends camera images trhough a Node connection
//

#include <iostream>
#include <memory>
#include <chrono>
#include <thread>

#include <HAL/Camera/CameraDevice.h>
#include <HAL/Utils/StringUtils.h>
#include <calibu/cam/CameraRig.h>
#include <calibu/cam/CameraXml.h>
#include <PbMsgs/ImageArray.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/fill_image.h>
#include <image_transport/image_transport.h>

#define nodeName "roscam" //ROS node name
#define defaultURI "opencv://" //Default URI for camera data
#define defaultFPS 10 //Frames per second
#define defaultCameraModel "" //Path to camera model XML file


std::string findFormat(uint32_t format)
{
  switch (format)
    {
    case pb::PB_LUMINANCE:
      return sensor_msgs::image_encodings::MONO8;
    case pb::PB_RGB:
      return sensor_msgs::image_encodings::RGB8;
    case pb::PB_RGBA:
      return sensor_msgs::image_encodings::RGBA8;
    case pb::PB_RAW:
      return sensor_msgs::image_encodings::MONO8;
    case pb::PB_BGR:
      return sensor_msgs::image_encodings::BGR8;
    case pb::PB_BGRA:
      return sensor_msgs::image_encodings::BGRA8;
    default:
      ROS_FATAL("Unknown HAL image format: 0x%x\n", format);
      return NULL;
    }
}

uint8_t findBytesPerPixel(uint32_t format)
{
  switch (format)
    {
    case pb::PB_LUMINANCE:
    case pb::PB_RAW:
      return 1;
    case pb::PB_BGR:
    case pb::PB_RGB:
      return 3;
    case pb::PB_RGBA:
    case pb::PB_BGRA:
      return 4;
    default:
      ROS_FATAL("Unknown HAL image format: 0x%x\n", format);
      return 0;
    }
}

int main(int argc, char *argv[])
{

  std::string hal_uri;
  double fps;
  std::string camera_model;

  ros::init(argc, argv, nodeName);
  ros::NodeHandle nh("~");
  image_transport::ImageTransport it(nh);
  image_transport::CameraPublisher pub = it.advertiseCamera("image", 1);
 
  // get params
  hal_uri = defaultURI;
  fps = defaultFPS;
  camera_model = defaultCameraModel;

  nh.getParam("URI",hal_uri);
  nh.getParam("fps",fps);
  nh.getParam("camera_model",camera_model);

  //Try to find the camera model
  std::string modelFileName = hal::ExpandTildePath(camera_model);
  if (!hal::FileExists(modelFileName))
    {
      ROS_FATAL("Could not find camera model file: %s\n", modelFileName.c_str());
      return -1;
    }

  ROS_INFO("Opening camera model file: %s\n", modelFileName.c_str());
  calibu::CameraRig rig = calibu::ReadXmlRig( modelFileName);
  ROS_INFO("Found %lu camera models\n", rig.cameras.size());
  
  const calibu::CameraModel& camModel = rig.cameras[0].camera;

  //Retrieve the K matrix
  Eigen::Matrix<double,3,3> kMatrix = camModel.K();

  //Save the float64 K matrix out:
  double *kMatrixRaw = kMatrix.data();


  // initiate camera
  std::unique_ptr<hal::Camera> cam;
  try {
    cam.reset(new hal::Camera(hal::Uri(hal_uri)));
  } catch (...) {
    ROS_FATAL("Could not create camera from URI:%s\n ", hal_uri.c_str());
    return -1;
  }

  ROS_INFO("Opening HAL URI: %s at framerate: %1.1f\n", hal_uri.c_str(), fps);


  // publish images
  ros::Rate loop_rate(fps);
  sensor_msgs::Image rosOut; //ros output

  while (nh.ok())
    {
      std::shared_ptr<pb::ImageArray> vImages = pb::ImageArray::Create();

      // Get next image from HAL
      cam->Capture(*vImages);
	  
      //Translate the pb::ImageArray into ROS message types
	  
      pb::CameraMsg camMsg = vImages->Ref();
      for (int ii = 0; ii < camMsg.image_size(); ++ii)
	{
	  pb::ImageMsg rawImgMsg = camMsg.image(ii); //hal input
	  pb::Image rawImg = pb::Image(rawImgMsg);

	  const uint8_t* rawData = (const unsigned char*) rawImg.data();
	  uint32_t height = rawImgMsg.height();
	  uint32_t width = rawImgMsg.width();


	  fillImage(rosOut, findFormat(rawImgMsg.format()).c_str(), 
		    height, width, 
		    findBytesPerPixel(rawImgMsg.format())*width,
		    rawData);

	  //Add timestamp
	  rosOut.header.stamp = ros::Time::now();

	  // Push the camera info


	  sensor_msgs::CameraInfo camera_info;
	  camera_info.header.frame_id = rosOut.header.frame_id;
	  camera_info.width = rawImgMsg.width();
	  camera_info.height = rawImgMsg.height();
	  memcpy(&camera_info.K,  kMatrixRaw, 9*sizeof(*kMatrixRaw));
	  camera_info.header.frame_id = rosOut.header.frame_id;
	  camera_info.header.stamp = rosOut.header.stamp;
		
	  pub.publish(rosOut, camera_info);
	  ros::spinOnce();
	  loop_rate.sleep();
	}

    }
  return 0;
}
