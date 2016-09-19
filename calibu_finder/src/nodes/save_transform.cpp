#include <fstream>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>

#include <geometry_msgs/TransformStamped.h>

#include <Eigen/Dense>

using std::string;
using std::cout;
using std::endl;

using boost::shared_ptr;
using boost::make_shared;


FILE* outputFile;

void executeCallback(const geometry_msgs::TransformStamped::ConstPtr& src)
{

  
    const geometry_msgs::Quaternion quat =  src->transform.rotation;
    
    const geometry_msgs::Vector3 trans =  src->transform.translation;
    

    /* Write the output line */
    ros::Time curTime = src->header.stamp;
    fprintf(outputFile, "%d.%09d, %0.12f, %0.12f,%0.12f", curTime.sec, curTime.nsec, trans.x,
	    trans.y, trans.z);
    fprintf(outputFile, ",%0.12f, %0.12f,%0.12f,%0.12f\n",
	    quat.x,
	    quat.y,
	    quat.z,
	    quat.w);

    return;
   
}


int main(int argc, char** argv)
{
  std::string trans_topic_;

  ros::Subscriber subs;
  ros::init(argc, argv, "trans_save", ros::init_options::AnonymousName );
  ros::NodeHandle pnh("~");

  
  if(!pnh.getParam("trans_topic", trans_topic_)){
    trans_topic_ = "/pose";
  }


  ROS_INFO("Writing to dir: [%s]", getenv("PWD"));
  ROS_INFO("Reading transforms from [%s]", trans_topic_.c_str());

  char fileName[64];
  std::string outputPath = getenv("PWD");
  
  sprintf(fileName, "%s/transform_poses.csv", outputPath.c_str());
  outputFile = fopen(fileName, "w");

  
  //Subscribe to the transform stream
  subs = pnh.subscribe(trans_topic_, 1000, &executeCallback);
  while (ros::ok())
    {
      ros::spinOnce();
    }
  fclose(outputFile);
  
  return 0;
}
