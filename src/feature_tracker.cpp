/* OpenCV-based feature tracker

   Subscribes to a ROS image topic, transforms to openCV, extracts features, runs pnp, publishes a set of:
   1. features
   a. featureID
   b. x/y pos

   2. Current 6DOF pose as estimated by pnpRansac (?)

   Code based on the pose_estimation sample from OpenCV's calib3d module:
https://github.com/Itseez/opencv/blob/master/samples/cpp/tutorial_code/calib3d/real_time_pose_estimation/src/main_detection.cpp
 https://github.com/Itseez/opencv/blob/master/samples/cpp/lkdemo.cpp

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
  
  cv::Mat prevGray;

  uint32_t frameCount;
  
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
  const uint16_t MAX_TRACKS=100;
  const uint16_t MIN_TRACKS=75; 
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

  frameCount = 0;
  
  indexParams = cv::makePtr<cv::flann::LshIndexParams>(6, 12, 1); // instantiate LSH index parameters
  searchParams = cv::makePtr<cv::flann::SearchParams>(50);       // instantiate flann search parameters

  // instantiate FlannBased matcher
  matcher = cv::makePtr<cv::FlannBasedMatcher>(indexParams, searchParams);
  rmatcher.setDescriptorMatcher(matcher);                                                         // set matcher
  rmatcher.setRatio(ratioTest); // set ratio test parameter

  arcID = 0;
  
  //Advertise the services
  arcService = n_.advertiseService("GetArcs", &FeatureTracker::serviceCallback, this);
  clearService = n_.advertiseService("ClearArcs", &FeatureTracker::clearArcsCallback, this);
}

float FeatureTracker::euclideanDist(const cv::Point& p, const cv::Point& q) {
  cv::Point diff = p - q;
    return cv::sqrt(diff.x*diff.x + diff.y*diff.y);
}

void FeatureTracker::process(const sensor_msgs::ImageConstPtr& msg)
{
  cv::TermCriteria termcrit(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,10,0.03);
  cv::Size subPixWinSize(10,10), winSize(5,5);
  
  cv_bridge::CvImagePtr src_ptr;
  cv_bridge::CvImagePtr dest_ptr(new cv_bridge::CvImage);
  imageHeight = msg->height;
  imageWidth = msg->width;

  vector<uint8_t> status;
  vector<float> err;
  
  try {
    src_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    //Source Mat is available at src_ptr->image
    cv::Mat graySrc;
    cv::cvtColor(src_ptr->image, graySrc, cv::COLOR_BGR2GRAY);
    if (frameCount == 0)
      {
	// automatic initialization: src + dest are the same thing
	vector<cv::Point2f> trackedPoints;
	cv::goodFeaturesToTrack(graySrc, trackedPoints, FeatureTracker::MAX_TRACKS, 0.5, 15, cv::Mat(), 3, 0, 0.04);
	cv::cornerSubPix(graySrc, trackedPoints, subPixWinSize, cv::Size(-1,-1), termcrit);
	//For this run, there will be this many features (no more)
	
	//Save the image as the previous image
	graySrc.copyTo(prevGray);
	frameCount++;
	int pointIndex = 0;
	for (int i=0; i<trackedPoints.size(); i++)
	  {
	    shared_ptr<ArcInfo> newArc = make_shared<ArcInfo>();
	    //All arcs are now active!
	    newArc->active = true;
	    newArc->id = i;
	    newArc->points.push_back(trackedPoints[i]);
	    arcs.push_back(newArc);

	  }

	/*
	if (display_)
	  {
	    cv::Mat featImage;
	    cv::Scalar featColor = cv::Scalar(255, 0, 0);
	    src_ptr->image.copyTo(featImage);
	    for (int i=0; i<  trackedPoints.size(); i++)
	      {
		//ROS_INFO("Drawing point %d",i++);
		cv::circle(featImage, trackedPoints[i], 4, featColor, -1, 8); 
	      }
	    cv::imshow("Initial features", featImage);
	  }
	*/
	
	ROS_INFO("Initialization complete with %d features!", trackedPoints.size());
	return;
      }

    //The input vector is the last point from each active arc
    vector<cv::Point2f> pointsToTrack;
    for (shared_ptr<ArcInfo> thisArc : arcs)
    {
      if (thisArc->active)
	{
	  pointsToTrack.push_back(thisArc->points.back()); //use last position
	}
    }

    ROS_INFO("Tracking %d active points", pointsToTrack.size());
    if (pointsToTrack.size() == 0)
      {
	ROS_INFO("No points to track, returning");
	return;
      }


    
    //We now have a set of features in trackedPoints, and an old image in prevGray, compute the flow into the current image
    vector<cv::Point2f> newTracks;
    cv::calcOpticalFlowPyrLK(prevGray, graySrc, pointsToTrack, newTracks, status, err, winSize, 1, termcrit, 0, 0.001);

    /*
    if (display_)
	  {
	    cv::Mat trackedImage;
	    cv::Scalar featColor = cv::Scalar(0, 255, 0);
	    src_ptr->image.copyTo(trackedImage);
	    for (int i=0; i<  newTracks.size(); i++)
	      {
		if (status[i] == 1)
		  {
		    //ROS_INFO("Found a good track to draw (%d)", i);
		    //ROS_INFO("Point %d: %f,%f", i, newTracks[i].x, newTracks[i].y);
		    cv::circle(trackedImage, newTracks[i], 4, featColor, -1, 8);
		  }
	      }
	    cv::imshow("Tracked features", trackedImage);
	  }
    */
    
    
    //Each status[i] indicates if the feature has been found (again)
    //Go through the arcs and append if was found, mark inactive otherwise
    int pointIndex = 0;
    int activePoints = 0;

    for (shared_ptr<ArcInfo> thisArc : arcs)
    {
      
      if (pointIndex > (newTracks.size() -1))
	{
	  //The remainder of the arcs weren't found
	  thisArc->active = false;
	  continue;
	}
      
      if (thisArc->active)
	{
	  //This arc participated in the search in the first place
	  if (status[pointIndex] == 1) //feature found!
	    {
	      float closeEps = 0.3f;
	      float farEps = 10.0f;
	      if ((euclideanDist(thisArc->points.back(), newTracks[pointIndex]) > closeEps) &&
		  (euclideanDist(thisArc->points.back(), newTracks[pointIndex]) < farEps))
		{
		  //Only include moving points (but not too much motion)
		  activePoints++;
		  thisArc->points.push_back(newTracks[pointIndex]);
		}
	    }
	  else
	    {
	      //Not found, mark as inactive
	      thisArc->active=false;
	    }

	}
      pointIndex++;
    }

    ROS_INFO("Found %d good correspondences", activePoints);
    //Save the current image as the past
    graySrc.copyTo(prevGray);


    //Do we need more tracks?
    int addedPoints = 0;
    if (activePoints < FeatureTracker::MIN_TRACKS)
      {
	vector<cv::Point2f> trackedPoints;
	cv::goodFeaturesToTrack(graySrc, trackedPoints, FeatureTracker::MAX_TRACKS, 0.5, 15, cv::Mat(), 3, 0, 0.04);
	cv::cornerSubPix(graySrc, trackedPoints, subPixWinSize, cv::Size(-1,-1), termcrit);
		
	int pointIndex = 0;
	float closeEps = 1.0f;
	for (int i=0; i<trackedPoints.size(); i++)
	  {
	    //Check to see if we're already tracking it
	    bool foundActive = false;
	    for (shared_ptr<ArcInfo> thisArc : arcs)
	      {
		if ((thisArc->active) && (euclideanDist(thisArc->points.back(), trackedPoints[i]) < closeEps))
		  {
		    foundActive = true; //ignore it
		  }
	      }

	    //Not being tracked, add it
	    if (!foundActive)
	      {
		shared_ptr<ArcInfo> newArc = make_shared<ArcInfo>();
		newArc->active = true;
		newArc->id = i;
		newArc->points.push_back(trackedPoints[i]);
		arcs.push_back(newArc);
		addedPoints++;
	      }
	  }
      }

    ROS_INFO("Added %d new points to track", addedPoints);
    //prepare the display
    dest_ptr->image = src_ptr->image.clone();
    cv::RNG rng(12345);
    //Draw all of the tracked features rng.uniform(0,255)
    cv::Scalar featColor = cv::Scalar(0, 0, 255);
    int arcsDrawn = 0;

    
    int count = 0;
    for (int i=0; i<  newTracks.size(); i++)
      {
	//ROS_INFO("Drawing point %d",i++);
	cv::circle(dest_ptr->image, newTracks[i], 4, featColor, -1, 8); 
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
      for (cv::Point2f thisPoint : thisArc->points)
	{
	  newArc.x.push_back(thisPoint.x);
	  newArc.y.push_back(thisPoint.y);
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
