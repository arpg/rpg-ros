

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>
#include <actionlib/server/simple_action_server.h>
#include <calibu/target/TargetGridDot.h>
#include <calibu/pose/Tracker.h>

//Message defs
#include <calibu_finder/conic_locator.h> 
#include <calibu_finder/conicData.h>
#include <calibration_msgs/CalibrationPoints.h>
#include <calibu_finder/ConfigAction.h>

/* This node is designed to mimic the image_cb_detector node out of the calibration package 

It supports an actionserver component to facilitate setting new parameters for the target finder
In this case, the Calibu finder only needs to know what target to look for  (a single String)

 */
using std::string;
using std::cout;
using std::endl;

using boost::shared_ptr;
using boost::make_shared;

using namespace calibu_finder;
using namespace calibration_msgs;

class ImageConicDetector 
{
public:
  ImageConicDetector(ros::NodeHandle nh);
  ~ImageConicDetector()  {  } ;

  void goalCallback();
  void preemptCallback();
  
private:
  void imageCallback(const sensor_msgs::ImageConstPtr& image);
  bool findConics(const sensor_msgs::ImageConstPtr& cal_image, CalibrationPoints* conics);
  ros::NodeHandle nh_;
  boost::mutex run_mutex_;
  actionlib::SimpleActionServer<calibu_finder::ConfigAction> as_;
  ros::Publisher pub_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber sub_;
  string target_type_;
  string image_topic_;
  int frame;
};

ImageConicDetector::ImageConicDetector(ros::NodeHandle nh)
  : as_("conic_detector_config", false), it_(nh_)
{
  
  nh_ = nh;
  ros::NodeHandle pnh("~");
  // camera parameters come from the request
  //Pull the image topic from the parameter server
  if(!pnh.getParam("image_topic", image_topic_)){
      image_topic_ = "/image_raw";
    }
  
  ROS_INFO("Reading image from [%s]", image_topic_.c_str());
  as_.registerGoalCallback( boost::bind(&ImageConicDetector::goalCallback, this) );
  as_.registerPreemptCallback( boost::bind(&ImageConicDetector::preemptCallback, this) );
  
  pub_ = nh_.advertise<CalibrationPoints>("features",1);
  sub_ = it_.subscribe(image_topic_, 2, boost::bind(&ImageConicDetector::imageCallback, this, _1));
  as_.start();
  
  ROS_INFO("Conic Detector started");
}

void ImageConicDetector::goalCallback()
  {

    boost::mutex::scoped_lock lock(run_mutex_);

    // Stop the previously running goal (if it exists)
    if (as_.isActive())
      as_.setPreempted();

    // Get the new goal from the action server
    calibu_finder::ConfigGoalConstPtr goal = as_.acceptNewGoal();
    assert(goal);

    //Pick off the new target type
    target_type_ = goal->target_name;
    ROS_INFO("Using target type: [%s]", target_type_.c_str());
    // Detect possible failure
    //    if (!success)
    //  as_.setAborted();
  }

void ImageConicDetector::preemptCallback()
  {
    boost::mutex::scoped_lock lock(run_mutex_);

    // Don't need to do any cleanup. Immeadiately turn it off
    as_.setPreempted();
  }

void ImageConicDetector::imageCallback(const sensor_msgs::ImageConstPtr& image)
  {
    boost::mutex::scoped_lock lock(run_mutex_);

    if (as_.isActive())
    {
     CalibrationPoints features;
      bool success;
      success = findConics(image, &features);

      if (!success)
      {
        ROS_ERROR("Error trying to detect Calibu target, not going to publish CalibrationPoints");
        return;
      }

      features.header.stamp = image->header.stamp; //so we can use a TimeSynchronizer to rehydrate features and original image
      pub_.publish(features);
    }
  }

bool ImageConicDetector::findConics(const sensor_msgs::ImageConstPtr& cal_image, CalibrationPoints *conics)
{

  //Suck an image out of the pipe:

  //ROS_INFO("Using target type [%s]", target_type_.c_str());
  
  calibu::TargetGridDot target( target_type_); // load a target dot grid from a preset

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

	conics->object_ids.push_back(conics_target_map[i]);
	geometry_msgs::Point image_point;
	image_point.x = tracker.GetConicFinder().Conics()[i].center[0];
	image_point.y = tracker.GetConicFinder().Conics()[i].center[1];
	image_point.z = 0;
			
	geometry_msgs::Point obj_point;
	obj_point.x = pos_3d[0];
	obj_point.y = pos_3d[1];
	obj_point.z = pos_3d[2];

	conics->object_points.push_back(obj_point);
	conics->image_points.push_back(image_point);

      }
  conics->success = conics->object_ids.size() ? 1 : 0;
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_conic_detector");
  ros::NodeHandle node_handle;
  ImageConicDetector conic_detector(node_handle);
  ros::spin();
  ros::waitForShutdown();
  return 0;
}
