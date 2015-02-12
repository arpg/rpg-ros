// Connect to a ROS Husky wheel odometry topic and publish encoder data to an ARPG Node 

#include <iostream>

#include <Node/Node.h>
#include <PbMsgs/Encoder.pb.h>

#include <ros/ros.h>
#include <ros/console.h>


#include "husky_msgs/HuskyWheelTick.h"

 // node publisher

node::node n;
std::string nodeTopic;

void odomCallback(const husky_msgs::HuskyWheelTick& msg)
{
  ROS_INFO("Odom message: [%s: %d] [%s: %d]", msg.name[0].c_str(), msg.tickCount[0],
	   msg.name[1].c_str(), msg.tickCount[1]);

  //Setup an Encoder protobuf
  pb::EncoderMsg nodeMsg;
  
  //Pass through ROS time from the original message
  double devTime = msg.header.stamp.sec + msg.header.stamp.nsec/1e9;
  nodeMsg.set_device_time(devTime);
  nodeMsg.add_label(msg.name[0]);
  nodeMsg.add_label(msg.name[1]);

  nodeMsg.add_data(msg.tickCount[0]);
  nodeMsg.add_data(msg.tickCount[1]);

  n.publish(nodeTopic, nodeMsg);
}


int main(int argc, char *argv[])
{
  std::string nodeName = "ros2node";

  ros::init(argc, argv, nodeName);
  ros::NodeHandle nh("~");

  // get params
  
  std::string exportTopic = "/wheel_tick";
  nh.getParam("topic",exportTopic);
  
  nodeTopic = "wheel_tick";
  nh.getParam("nodeTopic", nodeTopic);

  //Setup the Node publisher
  n.init(nodeName);
  n.advertise(nodeTopic);

  ROS_INFO("Publishing ROS topic [%s] through Node as [%s/%s]", exportTopic.c_str(), nodeName.c_str(), nodeTopic.c_str());
  
  ros::Subscriber sub = nh.subscribe(exportTopic, 1000, odomCallback);

  ros::spin();
  
  return 0;
}
