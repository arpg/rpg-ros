// This node knows about several types of ROS messages -
// pick one type via a parameter and it will publish via Node
// 1 message type per node instantiation
//Types so far:
// 1. Connect to a ROS Husky wheel odometry topic and publish encoder data to an ARPG Node 
// 2. Connect to a ROS Image topic and publish imagery to an ARPG Node

#include <iostream>

//ARPG includes
#include <Node/Node.h>
#include <PbMsgs/Encoder.pb.h>
#include <PbMsgs/Image.pb.h>
#include <PbMsgs/ImageArray.h>

//ROS includes
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/message.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/fill_image.h>
#include <image_transport/image_transport.h>

#include "husky_msgs/HuskyWheelTick.h"

 // node publisher

node::node n;
std::string ros_topic;
std::string node_topic;


/*Split fcn from http://stackoverflow.com/questions/236129/split-a-string-in-c */

using std::string;
using std::cout;
using std::endl;
using std::vector;

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

string findROSFormat(uint32_t format)
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

pb::Type findPbType(string format)
{
  if (format == sensor_msgs::image_encodings::MONO8)
    return pb::PB_UNSIGNED_BYTE;
  if (format == sensor_msgs::image_encodings::BGR8)
    return pb::PB_UNSIGNED_BYTE;
   if (format == sensor_msgs::image_encodings::RGB8)
    return pb::PB_UNSIGNED_BYTE;
  if (format == sensor_msgs::image_encodings::MONO16)
    return pb::PB_UNSIGNED_SHORT;
  
  ROS_FATAL("Unknown ROS image format: [%s]\n", format.c_str());
  return (pb::Type) 0;
}

pb::Format findPbFormat(string format)
{
  //Protobuf formats are typedefed ints, ROS formats are strings
  if (format == sensor_msgs::image_encodings::MONO8)
    return pb::PB_LUMINANCE;
  
  if (format == sensor_msgs::image_encodings::MONO16)
    return pb::PB_LUMINANCE;

  if (format == sensor_msgs::image_encodings::BGR8)
    return pb::PB_BGR;
  
  if (format == sensor_msgs::image_encodings::RGB8)
    return pb::PB_RGB;
  
  ROS_FATAL("Unknown ROS image format: [%s]\n", format.c_str());
  return (pb::Format) 0;
  /*
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
  */
  
}

void master_callback(const sensor_msgs::Image& msg)
{
  //this callback know how to handle several message types
}

sensor_msgs::ImageConstPtr last_rh;

void rh_callback(const sensor_msgs::ImageConstPtr& msg)
{
  last_rh = msg;
  //ROS_INFO("Got RH callback");
  
}

void lh_callback(const sensor_msgs::ImageConstPtr& msg)
{
  //ROS_INFO("Got LH callback");

  
  //Setup an CameraMsg protobuf for Node, so that we can embed multiple
  //camera images in a single Node message
  
  pb::CameraMsg camMsg;
  pb::ImageMsg *l_imgMsg = camMsg.add_image(); //add the left
  pb::Image l_rawImg = pb::Image(*l_imgMsg);
  
  pb::ImageMsg *r_imgMsg = camMsg.add_image(); //add the right
  pb::Image r_rawImg = pb::Image(*r_imgMsg);

  l_imgMsg->set_width(msg->width);
  r_imgMsg->set_width(last_rh->width);
  //r_imgMsg->set_width(100);
  
  l_imgMsg->set_height(msg->height);
  r_imgMsg->set_height(last_rh->height);
  // r_imgMsg->set_height(100);

  l_imgMsg->set_data(&msg->data[0], msg->step * msg->height);
  r_imgMsg->set_data(&last_rh->data[0], last_rh->step * last_rh->height);

  l_imgMsg->set_format(findPbFormat(msg->encoding));
  r_imgMsg->set_format(findPbFormat(last_rh->encoding));
  // r_imgMsg->set_format(findPbFormat("mono8"));
  
  l_imgMsg->set_type(findPbType(msg->encoding));
  r_imgMsg->set_type(findPbType(last_rh->encoding));		    
  //Pass through ROS time from the original message
  double devTime = msg->header.stamp.sec + msg->header.stamp.nsec/1e9;
  camMsg.set_device_time(devTime);

  //cout << "Published image pair" << endl;
  n.publish(node_topic, camMsg);
}

void imageCallback(const sensor_msgs::Image& msg)
{
  //Setup an Image protobuf
  pb::ImageMsg nodeMsg;
  
  //Pass through ROS time from the original message
  //double devTime = msg.header.stamp.sec + msg.header.stamp.nsec/1e9;
  //nodeMsg.set_device_time(devTime);
  
  std::string rawTopic = "wheel";
  n.publish(rawTopic, nodeMsg);
}

void encoderCallback(const husky_msgs::HuskyWheelTick& msg)
{
  //ROS_INFO("Raw message: [%s: %d] [%s: %d]", msg.name[0].c_str(), msg.tickCount[0],
  //	   msg.name[1].c_str(), msg.tickCount[1]);

  //Setup an Encoder protobuf
  pb::EncoderMsg nodeMsg;
  std::string rawTopic = "wheel";
  
  //Pass through ROS time from the original message
  double devTime = msg.header.stamp.sec + msg.header.stamp.nsec/1e9;
  nodeMsg.set_device_time(devTime);
  nodeMsg.add_label(msg.name[0]);
  nodeMsg.add_label(msg.name[1]);

  nodeMsg.add_data(msg.tickCount[0]);
  nodeMsg.add_data(msg.tickCount[1]);

  n.publish(rawTopic, nodeMsg);
}

void jsCallback(const sensor_msgs::JointState& msg)
{
  //ROS_INFO("JS message recv");
  std::string jsTopic = "topic";
  
  //Setup an Encoder protobuf
  pb::EncoderMsg nodeMsg;
  
  //Pass through ROS time from the original message
  double devTime = msg.header.stamp.sec + msg.header.stamp.nsec/1e9;
  nodeMsg.set_device_time(devTime);

  for (unsigned int i =0; i<msg.name.size(); i++)
    {
      nodeMsg.add_label(msg.name[i]);
      nodeMsg.add_data(msg.position[i]);
    }
  n.publish(jsTopic, nodeMsg);
}

int main(int argc, char *argv[])
{
  string nodeName = "ros2node";
  string mode = "stereo";
  
  ros::init(argc, argv, nodeName);
  ros::NodeHandle nh("~");

  // get params
  nh.getParam("mode", mode);
  //Mode is a listing of things that ros2node knows about

  
  nh.getParam("ros_topic",ros_topic);
  nh.getParam("node_topic", node_topic);

  if (ros_topic == "" || node_topic == "")
    {
      ROS_FATAL("Must provide both ROS and Node topics!");
      return false;
    }
  

  ROS_INFO("Publishing ROS topic [%s] through Node as [%s/%s]", ros_topic.c_str(), nodeName.c_str(), node_topic.c_str());
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber l_sub;
  image_transport::Subscriber r_sub;
  
  if (mode == "stereo")
    {
      // Stereo uses two subscribers and a shmutex to publish when either one gets updated

      //The ros_topic string is actually two strings, one for each topic, split by a pipe
      vector<string> results = split<string>(ros_topic, "|");

      if (results.size() != 2)
	{
	  ROS_FATAL("Stereo needs two topics split by a pipe");
	}
      else
	{
	  ROS_INFO("Using left-hand topic: [%s]", results[0].c_str());
	  ROS_INFO("Using right-hand topic: [%s]", results[1].c_str());
	}

      l_sub = it.subscribe(results[0], 1000, lh_callback);
      r_sub = it.subscribe(results[1], 1000, rh_callback);
    }
  else if (mode == "encoder")
    {
      //Odometry uses a single subscriber
      ros::Subscriber rs = nh.subscribe(ros_topic, 1000, encoderCallback);
    }
  
  //Setup the Node publisher
  n.init(nodeName);
  n.advertise(node_topic);


  ros::spin();
  
  return 0;
}
