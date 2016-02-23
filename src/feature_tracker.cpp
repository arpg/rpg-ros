/* OpenCV-based feature tracker

   Subscribes to a ROS image topic, transforms to openCV, extracts features, runs pnp, publishes a set of:
   1. features
   a. featureID
   b. x/y pos

   2. Current 6DOF pose as estimated by pnpRansac (?)

   Code based on the pose_estimation sample from OpenCV's calib3d module:
https://github.com/Itseez/opencv/blob/master/samples/cpp/tutorial_code/calib3d/real_time_pose_estimation/src/main_detection.cpp
 
and reuses modules as needed


 */
#include <ros/ros.h>

#include <feature_tracker/feature_tracker.h>
/*

Code from :
http://www.learnopencv.com/blob-detection-using-opencv-python-c/
http://opencv-srf.blogspot.com/2010/09/object-detection-using-color-seperation.html
 */

using namespace std;

namespace feature_tracker
{

class FeatureTracker
{
public:
  FeatureTracker();

  void process(const sensor_msgs::ImageConstPtr& msg);
private:
  ros::Publisher image_pub_;
  ros::NodeHandle n_;
  image_transport::ImageTransport it;
  image_transport::Subscriber image_sub;
  // Params
  uint8_t display_;
  float ratioTest;
  
  cv::Ptr<cv::FeatureDetector> orb;
  RobustMatcher rmatcher; // instantiate RobustMatcher
  cv::Ptr<cv::DescriptorMatcher> matcher;
  cv::Ptr<cv::flann::IndexParams> indexParams;
  cv::Ptr<cv::flann::SearchParams> searchParams;

  std::vector<std::shared_ptr<FrameInfo>> frames;

};

}

using namespace feature_tracker;


FeatureTracker::FeatureTracker() :
  it(n_)
{

  image_sub = it.subscribe("/image", 1, &FeatureTracker::process, this);

  ros::NodeHandle local_ns_("~");


  display_ = local_ns_.param("display", 0);
  ROS_INFO("[display]: %i", display_);

  ratioTest = local_ns_.param("ratio_test", 0.70f);
  ROS_INFO("[ratio_test]: %f", ratioTest);

  if (display_)
    {
      ROS_INFO("Displaying results");
    }

  indexParams = cv::makePtr<cv::flann::LshIndexParams>(6, 12, 1); // instantiate LSH index parameters
  searchParams = cv::makePtr<cv::flann::SearchParams>(50);       // instantiate flann search parameters

  // instantiate FlannBased matcher
  matcher = cv::makePtr<cv::FlannBasedMatcher>(indexParams, searchParams);
  rmatcher.setDescriptorMatcher(matcher);                                                         // set matcher
  rmatcher.setRatio(ratioTest); // set ratio test parameter
  
}

void FeatureTracker::process(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr src_ptr;
  cv_bridge::CvImagePtr dest_ptr(new cv_bridge::CvImage);

  
  try {
    //Find the color blob, use as a mask over the source color image for the result
    src_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    
    //Source Mat is available at src_ptr->image
    vector<cv::KeyPoint> keypoints_scene;  // to obtain the 2D points of the scene
    cv::Mat descriptors_scene;

    shared_ptr<FrameInfo> newFrame = make_shared<FrameInfo>();
    newFrame->seq = msg->header.seq;
    
    //Compute the descriptors to match for the next frame
    rmatcher.computeDescriptors(src_ptr->image, newFrame->keypoints, newFrame->descriptors);

    //If there is a past frame, try to match against its descriptors
    shared_ptr<FrameInfo> lastFrame = frames.back();
    if (lastFrame != NULL)
      {
	//Compute a list of matches from t-1 to t, stored in t
	rmatcher.robustMatchFromPrior(newFrame->matches, newFrame->descriptors, lastFrame->descriptors);
      }
    //Add to stored frame list
    frames.push_back(newFrame);
    
    //prepare the display
    dest_ptr->image = src_ptr->image.clone();

    /*
    for (int i=0; i<newFrame->keypoints.size(); i++)
      {
	cv::circle(dest_ptr->image, newFrame->keypoints[i].pt, 4, cv::Scalar(254,0,0), -1, 8);
      }
    */
    
    if(display_)
      {
	cv::imshow("Features", dest_ptr->image);
      }

    //Next step: Build up a descriptor database and try to find matches between frames
    //Be able to show tracks like sdtrack
    
    if (display_)
      cv::waitKey(1);
    
    // Send the annotated image over ROS
    src_ptr->encoding = sensor_msgs::image_encodings::BGR8;
    
    //image_pub_.publish(src_ptr->toImageMsg());
  } catch(cv_bridge::Exception & e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "feature_tracker");

  //ROS_INFO("Starting feature_tracker");
  FeatureTracker seg;


  ros::spin();
  return 0;
}
