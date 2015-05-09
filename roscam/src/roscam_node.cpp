// Sends camera images through a ROS connection
//

#include <iostream>
#include <memory>
#include <chrono>
#include <thread>
#include <vector>
#include <string>

#include <HAL/Camera/CameraDevice.h>
#include <HAL/Utils/StringUtils.h>
#include <calibu/Calibu.h>
#include <calibu/cam/camera_rig.h>
#include <calibu/cam/camera_xml.h>

#include <HAL/Image.pb.h>
#include <HAL/Messages/ImageArray.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/fill_image.h>
#include <image_transport/image_transport.h>

#define nodeName "roscam" //ROS node name
#define defaultURI "opencv://" //Default URI for camera data
#define defaultFPS 10 //Frames per second
#define defaultCameraModel "" //Path to camera model XML file


using namespace std;

struct hal_camera
{
  double kMatrix[9];
  uint32_t height;
  uint32_t width;
  image_transport::CameraPublisher pub;
};

/*Split fcn from http://stackoverflow.com/questions/236129/split-a-string-in-c */
template<typename T>
vector<T> split(const T & str, const T & delimiters)
  {
    vector<T> v;
    typename T::size_type start = 0;
    size_t pos = str.find_first_of(delimiters, start);
    while(pos != T::npos) {
      if(pos != start) // ignore empty tokens
	v.push_back(str.substr(start, pos - start));
      start = pos + 1;
      pos = str.find_first_of(delimiters, start);
    }
    if(start < str.length()) // ignore trailing delimiter
      v.push_back(str.substr(start, pos - start)); // add what's left of the string
    return v;
  }

std::string findFormat(uint32_t format, hal::Type imageType )
{
  switch (format)
    {
    case hal::PB_LUMINANCE:
      if (imageType == hal::PB_UNSIGNED_BYTE)
	return sensor_msgs::image_encodings::MONO8;
      else
	return sensor_msgs::image_encodings::MONO16;
      
    case hal::PB_RGB:
      return sensor_msgs::image_encodings::RGB8;
    case hal::PB_RGBA:
      return sensor_msgs::image_encodings::RGBA8;
    case hal::PB_RAW:
      return sensor_msgs::image_encodings::MONO8;
    case hal::PB_BGR:
      return sensor_msgs::image_encodings::BGR8;
    case hal::PB_BGRA:
      return sensor_msgs::image_encodings::BGRA8;
    default:
      ROS_FATAL("Unknown HAL image format: 0x%x type: 0x%x\n", format, imageType);
      return NULL;
    }
}

uint8_t findBytesPerPixel(uint32_t format, hal::Type imageType)
{
  switch (format)
    {
    case hal::PB_RAW:
      return 1;
    case hal::PB_LUMINANCE:
      if (imageType == hal::PB_UNSIGNED_BYTE)
	return 1;
      if (imageType == hal::PB_UNSIGNED_SHORT)
	return 2;
      
    case hal::PB_BGR:
    case hal::PB_RGB:
      return 3;
    case hal::PB_RGBA:
    case hal::PB_BGRA:
      return 4;
    default:
      ROS_FATAL("Unknown HAL image format: 0x%x type: 0x%x\n", format, imageType);
      return 0;
    }
}

int main(int argc, char *argv[])
{

  std::string hal_uri;
  double fps;
  std::string camera_model;
  std::string topicNames;
  
  ros::init(argc, argv, nodeName);
  ros::NodeHandle nh;
  ros::NodeHandle p_nh("~");
  image_transport::ImageTransport it(nh);
  vector<string> topics;
  vector<hal_camera*> cameras;
  
  // get params
  hal_uri = defaultURI;
  fps = defaultFPS;
  camera_model = defaultCameraModel;
  topicNames = "/image_raw";
  
  p_nh.getParam("URI",hal_uri);
  p_nh.getParam("fps",fps);
  p_nh.getParam("camera_model",camera_model);
  p_nh.getParam("topics",topicNames);
  //Try to find the camera model
  
  std::string modelFileName = hal::ExpandTildePath(camera_model);
  if (!hal::FileExists(modelFileName))
    {
      ROS_FATAL("Could not find camera model file: %s\n", modelFileName.c_str());
      return -1;
    }

  ROS_INFO("Opening camera model file: %s\n", modelFileName.c_str());

  std::shared_ptr<calibu::Rig<double>> rig = calibu::ReadXmlRig( modelFileName);
  
  ROS_INFO("Found %lu camera models\n", rig->cameras_.size());

  topics = split<string>(topicNames, "+");

  // initiate HAL camera
  std::unique_ptr<hal::Camera> cam;
  try {
    cam.reset(new hal::Camera(hal::Uri(hal_uri)));
  } catch (...) {
    ROS_FATAL("Could not create camera from URI:%s\n ", hal_uri.c_str());
    return -1;
  }

  ROS_INFO("Opening HAL URI: %s at framerate: %1.1f\n", hal_uri.c_str(), fps);
  ROS_INFO("Found camera with %lu channels\n", cam->NumChannels());

  if (cam->NumChannels() != rig->cameras_.size())
    {
      //The cam is publishing a different number of images than the rig file defines
      ROS_FATAL("Different number of cam channels and cam models!");
    }


  //Set up ROS publishers
  for (unsigned int i=0; i< topics.size(); i++)
    {
      ROS_INFO("Publishing channel [%d] as [%s]\n", i, topics[i].c_str());
      hal_camera* newCam = new hal_camera;
      newCam->pub = it.advertiseCamera(topics[i], 1);
      
       //Retrieve the K matrix
      std::shared_ptr<calibu::CameraInterface<double>> camModel = rig->cameras_[i];
      Eigen::Matrix<double,3,3> kMatrix = camModel->K();
      //Save the float64 K matrix out:
      double *kMatrixRaw = kMatrix.data();
      memcpy(&newCam->kMatrix[0], kMatrixRaw, 9*sizeof(*kMatrixRaw));
      newCam->height = cam->Width(i);
      newCam->width = cam->Height(i);
      cameras.push_back(newCam);
    }

  // publish images
  ros::Rate loop_rate(fps);
  sensor_msgs::Image rosOut; //ros output

  //Get one round of images from HAL to set up the publishers
  
  while (nh.ok())
    {
      std::shared_ptr<hal::ImageArray> vImages = hal::ImageArray::Create();

      // Get next image set from HAL
      cam->Capture(*vImages);
	  
      //Translate the hal::ImageArray into ROS message types
	  
      hal::CameraMsg camMsg = vImages->Ref();
      for (int ii = 0; ii < camMsg.image_size(); ++ii)
	{
	  hal::ImageMsg rawImgMsg = camMsg.image(ii); //hal input
	  hal::Image rawImg = hal::Image(rawImgMsg);

	  const uint8_t* rawData = (const unsigned char*) rawImg.data();
	  uint32_t height = rawImgMsg.height();
	  uint32_t width = rawImgMsg.width();

	  string imgFormat =  findFormat(rawImgMsg.format(), rawImgMsg.type());
	  uint32_t bytesPP =  findBytesPerPixel(rawImgMsg.format(), rawImgMsg.type());
	  fillImage(rosOut, imgFormat.c_str(), 
		    height, width, 
		    bytesPP*width,
		    rawData);

	  //TODO: Fill timestamp from the HAL source
	  
	  rosOut.header.stamp.sec = (uint64_t) rawImgMsg.timestamp();
	  rosOut.header.stamp.nsec = (rawImgMsg.timestamp() - rosOut.header.stamp.sec) * 1e9;


	  // Push the camera info


	  sensor_msgs::CameraInfo camera_info;
	  camera_info.header.frame_id = rosOut.header.frame_id;
	  camera_info.width = rawImgMsg.width();
	  camera_info.height = rawImgMsg.height();
	  memcpy(&camera_info.K,  &cameras[ii]->kMatrix[0], 9*sizeof(double));
	  camera_info.header.frame_id = rosOut.header.frame_id;
	  camera_info.header.stamp = rosOut.header.stamp;

	  //Push both the image and its associated camera_info
	  cameras[ii]->pub.publish(rosOut, camera_info);
	  
	}
      ros::spinOnce();
      //loop_rate.sleep();
    }
  return 0;
}
