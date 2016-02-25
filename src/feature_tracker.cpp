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
  std::vector<std::shared_ptr<ArcInfo>> arcs;
  
  float euclideanDist(const cv::Point& p, const cv::Point& q);
  bool serviceCallback(feature_tracker::GetArcs::Request &req, feature_tracker::GetArcs::Response &res);
  bool clearArcsCallback(feature_tracker::ClearArcs::Request &req, feature_tracker::ClearArcs::Response &res);
  ros::ServiceServer arcService;
  ros::ServiceServer clearService;
  int arcID;
  uint16_t imageWidth;
  uint16_t imageHeight;
  
};

}

using namespace feature_tracker;


FeatureTracker::FeatureTracker() :
  it(n_)
{

  image_sub = it.subscribe("image", 1, &FeatureTracker::process, this);

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

  arcID = 0;

  //Advertise the service
  arcService = n_.advertiseService("GetArcs", &FeatureTracker::serviceCallback, this);
  clearService = n_.advertiseService("ClearArcs", &FeatureTracker::clearArcsCallback, this);
}

float FeatureTracker::euclideanDist(const cv::Point& p, const cv::Point& q) {
  cv::Point diff = p - q;
    return cv::sqrt(diff.x*diff.x + diff.y*diff.y);
}

void FeatureTracker::process(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr src_ptr;
  cv_bridge::CvImagePtr dest_ptr(new cv_bridge::CvImage);
  imageHeight = msg->height;
  imageWidth = msg->width;
  
  try {
    //Find the color blob, use as a mask over the source color image for the result
    src_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    
    //Source Mat is available at src_ptr->image
    vector<cv::KeyPoint> keypoints_scene;  // to obtain the 2D points of the scene
    cv::Mat descriptors_scene;

    shared_ptr<FrameInfo> newFrame = make_shared<FrameInfo>();
    newFrame->seq = msg->header.seq;
    
    //Compute the descriptors to match for the next frame
    rmatcher.computeKeyPoints(src_ptr->image, newFrame->keypoints);
    rmatcher.computeDescriptors(src_ptr->image, newFrame->keypoints, newFrame->descriptors);

    //ROS_INFO("Found %d keypoints", newFrame->keypoints.size());
    //If there is a past frame, try to match against its descriptors
  
    if (frames.size() != 0  )
      {
	shared_ptr<FrameInfo> lastFrame = frames.back();
	//Compute a list of matches from t-1 to t, stored in t
	rmatcher.robustMatchFromPrior(newFrame->matches, newFrame->descriptors, lastFrame->descriptors);
	//ROS_INFO("Found %d matches", newFrame->matches.size());

	//ROS_INFO("Number of active arcs: %d", arcs.size());
	//Update the arc info
	//go through the list of active arcs and mark all of them as untouched
	for (shared_ptr<ArcInfo> thisArc : arcs)
	  {
	    thisArc->touched  = false;
	  }
	
	for (cv::DMatch thisMatch : newFrame->matches)
	  {
	    //go through the list of active arcs and touch those that are referenced
	    int foundExisting = false;
	    ROS_INFO("Looking for trainingIdx: %d", thisMatch.trainIdx);
	    
	    for (shared_ptr<ArcInfo> thisArc : arcs)
	      {
		//ROS_INFO("Processing arcID: %d", thisArc->id);
		//ROS_INFO("Its last index was: %d", thisArc->matchIDs.back());
		//Compare the training (old) against the query (new)
		if (thisArc->active && (thisArc->matchIDs.back() == thisMatch.trainIdx))
		
		  {
		    //Check the distance between the points - make sure it moved somewhat
		    if (euclideanDist(newFrame->keypoints[thisMatch.queryIdx].pt,
					  lastFrame->keypoints[thisMatch.trainIdx].pt) > 2)
		      {
			ROS_INFO("Adding point to arc %d", thisArc->id);
			thisArc->touched = true;
			foundExisting = true;
			thisArc->points.push_back(newFrame->keypoints[thisMatch.queryIdx]);
			thisArc->matchIDs.push_back(thisMatch.queryIdx); //so we can match against it
			//during the next round
			continue; //since it can't match any others
		      }
		  }
	      }
	    
	    if (!foundExisting)
	      {
		// Add a new arc to look for
		shared_ptr<ArcInfo> newArc = make_shared<ArcInfo>();
		newArc->id = arcID++;
		newArc->touched = true;
		newArc->active = true;
		newArc->points.push_back(newFrame->keypoints[thisMatch.queryIdx]);
		newArc->matchIDs.push_back(thisMatch.queryIdx);
		//ROS_INFO("Added arcID: %d", newArc->id);
		arcs.push_back(newArc);
	      }
	  }
	
	//Any arc that wasn't touched in this round is now inactive
	//go through the list of active arcs and mark all of them as untouched
	for (vector<shared_ptr<ArcInfo>>::iterator it = arcs.begin(); it != arcs.end();)
	  {
	    shared_ptr<ArcInfo> thisArc = *it;
	    if (!thisArc->touched)
	      {
		thisArc->active = false;
		//Only save inactive arcs where the length is > 3 frames 
		if (thisArc->points.size() < 3)
		  {
		    //Remove from the list
		    arcs.erase(it);
		  }
		else
		  it++;
	      }
	    else
	      it++;	    
	  }
	
      }
 
    //Add to stored frame list
    frames.push_back(newFrame);
    
    //prepare the display
    dest_ptr->image = src_ptr->image.clone();
    cv::RNG rng(12345);
    //Draw all of the arcs
    int arcsDrawn = 0;
    for (shared_ptr<ArcInfo> thisArc : arcs)
      {
	if (thisArc->points.size() < 8)
	  continue;
	//if (arcsDrawn > 20)
	//  break;
	//Only draw the longest of arcs
	cv::Scalar arcColor = cv::Scalar(0, 0, rng.uniform(0,255));
	for (cv::KeyPoint thisPoint : thisArc->points)
	  {
	    cv::circle(dest_ptr->image, thisPoint.pt, 1, arcColor, -1, 8); 
	  }
	arcsDrawn++;
	ROS_INFO("Arc length (frames): %d", thisArc->points.size());
      }
    


    
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
bool FeatureTracker::serviceCallback(feature_tracker::GetArcs::Request &req, feature_tracker::GetArcs::Response &res)
{
  res.imageWidth = imageWidth;
  res.imageHeight = imageHeight;
  
  //Process the arc list and return a set of ordered points
  for (shared_ptr<ArcInfo> thisArc : arcs)
    {
      if (thisArc->points.size() < req.minSize)
	continue;
      Arc newArc;
      newArc.id = thisArc->id;
      for (cv::KeyPoint thisPoint : thisArc->points)
	{
	  newArc.x.push_back(thisPoint.pt.x);
	  newArc.y.push_back(thisPoint.pt.y);
	}
      res.arcs.push_back(newArc);
    }
  return true;
}

bool FeatureTracker::clearArcsCallback(feature_tracker::ClearArcs::Request &req, feature_tracker::ClearArcs::Response &res)
{
  arcs.clear();
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "feature_tracker");

  //ROS_INFO("Starting feature_tracker");
  FeatureTracker seg;

  while (ros::ok())
    {
        ros::spinOnce();
	cv::waitKey(1);
    }

  return 0;
}
