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

#include <feature_tracker/feature_tracker.h>

/*

Code from :
http://www.learnopencv.com/blob-detection-using-opencv-python-c/
http://opencv-srf.blogspot.com/2010/09/object-detection-using-color-seperation.html
rpg_svo's KLT initializer 

 */

using namespace std;

namespace feature_tracker
{

class FeatureTracker
{
public:
  FeatureTracker();
  ~FeatureTracker();
  void process(const sensor_msgs::ImageConstPtr& msg);
private:
  ros::Publisher image_pub_;
  ros::Publisher arc_pub_;
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
  void pruneKeypoints(vector<cv::KeyPoint> keyPoints, vector<cv::Point2f> &points);
  void getFeatures( cv::Mat &src, vector<cv::Point2f> &trackedPoints);
  float euclideanDist(const cv::Point& p, const cv::Point& q);
  bool serviceCallback(feature_tracker::GetArcs::Request &req, feature_tracker::GetArcs::Response &res);
  bool clearArcsCallback(feature_tracker::ClearArcs::Request &req, feature_tracker::ClearArcs::Response &res);
  void publishArcs(uint16_t width, uint16_t height);
  ros::ServiceServer arcService;
  ros::ServiceServer clearService;
  int arcID;
  uint16_t imageWidth;
  uint16_t imageHeight;
  const uint16_t MAX_TRACKS=100;
  const uint16_t MIN_TRACKS=75;
  mutex queueLock;
  thread *serviceThread;
  queue<boost::shared_ptr<const sensor_msgs::Image>> imageQueue;
  void service();
  void processOne(const sensor_msgs::ImageConstPtr& msg);
};

}

using namespace feature_tracker;

FeatureTracker::~FeatureTracker()
{
}

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

  //Service thread
  serviceThread = new thread(bind(&FeatureTracker::service, this));

  //Annotated publisher
  image_pub_ = n_.advertise<sensor_msgs::Image>("features", 1);

  //Annotated publisher
  arc_pub_ = n_.advertise<sensor_msgs::Image>("arcs", 1);

}

float FeatureTracker::euclideanDist(const cv::Point& p, const cv::Point& q) {
  cv::Point diff = p - q;
    return cv::sqrt(diff.x*diff.x + diff.y*diff.y);
}

void FeatureTracker::pruneKeypoints(vector<cv::KeyPoint> keyPoints, vector<cv::Point2f> &points)
{
  //  if (keyPoints.size() < i = keyPoints.size() - FeatureTracker::MAX_TRACKS
  int pointsToAdd = 50;
  for (int i = 0; i < pointsToAdd; i++)
    {
      cv::KeyPoint key = keyPoints[i];
      //printf("Response: %1.2f\n", key.response);
      points.push_back(key.pt);
    }
}

void FeatureTracker::process(const sensor_msgs::ImageConstPtr& msg)
{
  queueLock.lock();
  imageQueue.push(msg);
  queueLock.unlock();
}

void FeatureTracker::service()
{
  ROS_INFO("FeatureTracker: Started service thread\n");
  sensor_msgs::ImageConstPtr thisMsg;
  while (true)
    {
      thisMsg = NULL;
      queueLock.lock();
      if (imageQueue.size() > 0)
	{
	  thisMsg = imageQueue.front();
	  imageQueue.pop();
	}
      else
	thisMsg = NULL;
      queueLock.unlock();

      if (thisMsg)
	{
	  //ROS_INFO("Processing message");
	  processOne(thisMsg);
	  thisMsg = NULL;
	}
      else
	{
	  //No message to process yet...
	  usleep(1000);
	}
      
    }
}

void FeatureTracker::getFeatures( cv::Mat &src, vector<cv::Point2f> &trackedPoints)
{
  //cv::goodFeaturesToTrack(src, trackedPoints, FeatureTracker::MAX_TRACKS, 0.2, 7, cv::Mat(), 7, 0, 0.04);
  //try FAST:

  /*
  cv::Size subPixWinSize(10,10);
  cv::TermCriteria termcrit(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,10,0.03);

    
  vector<cv::KeyPoint> trackedKeyPoints;
		
  cv::Ptr<cv::FeatureDetector> detector = cv::FeatureDetector::create("STAR");
  detector->detect(src, trackedKeyPoints);
  pruneKeypoints(trackedKeyPoints, trackedPoints);
  cv::cornerSubPix(src, trackedPoints, subPixWinSize, cv::Size(-1,-1), termcrit);
  */
  
  // Setup SimpleBlobDetector parameters.
  cv::SimpleBlobDetector::Params params;

  // Change thresholds
  //params.minThreshold = 10;
  //params.maxThreshold = 200;

  // Filter by Area.
  //params.filterByArea = false;
  //params.minArea = 1500;

  // Filter by Circularity
  //params.filterByCircularity = true;
  //params.minCircularity = 0.1;

  // Filter by Convexity
  //params.filterByConvexity = true;
  //params.minConvexity = 0.87;

  // Filter by Inertia
  //params.filterByInertia = true;
  //params.minInertiaRatio = 0.01;

  cv::SimpleBlobDetector detector(params);
  //detector.create("SimpleBlob");
  // Storage for blobs
  std::vector<cv::KeyPoint> keypoints;

  // Detect blobs
  detector.detect( src, keypoints);
  pruneKeypoints(keypoints, trackedPoints);
}

void FeatureTracker::processOne(const sensor_msgs::ImageConstPtr& msg)
{

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
	//Save the image as the previous image
	graySrc.copyTo(prevGray);

	vector<cv::Point2f> trackedPoints;
	getFeatures(graySrc, trackedPoints);

	int pointIndex = 0;
	for (int i=0; i<trackedPoints.size(); i++)
	  {
	    shared_ptr<ArcInfo> newArc = make_shared<ArcInfo>();
	    //New arcs are active by default
	    newArc->active = true;
	    newArc->id = i;
	    newArc->points.push_back(trackedPoints[i]);
	    arcs.push_back(newArc);
	  }

      }

    //The input vector is the last point seen from each active arc
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
    vector<cv::Point2f> newTracks; //cv::OPTFLOW_USE_INITIAL_FLOW
    cv::TermCriteria termcrit(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,30,0.01);
    cv::Size winSize(10,10);
  
    cv::calcOpticalFlowPyrLK(prevGray, graySrc, pointsToTrack, newTracks, status, err, winSize, 0, termcrit, 0, 0.001);

    
    //Each status[i] indicates if the feature has been found (again)
    //Go through the arcs and append if was found, mark inactive otherwise


    int trackedPoints = 0;
    for (uint8_t stat : status)
      {
	if (stat == 1)
	  {
	    trackedPoints++;
	  }
      }

    ROS_INFO("Found %d correspondences, filtering...", trackedPoints);
    
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
	  //Since the for..each enumerates the arcs in the same order as above
	  
	  if (status[pointIndex] == 1) //feature found!
	    {
	      //ROS_INFO("Point %d was tracked", pointIndex);
	      float closeEps = 0.0f;
	      float farEps = 5.0f;
	      if (euclideanDist(thisArc->points.back(), newTracks[pointIndex]) < farEps)
		{
		  //Only include moving points (but not too much motion)
		  activePoints++;
		  thisArc->points.push_back(newTracks[pointIndex]);
		}
	      else
		{
		  ROS_INFO("Discarding match, dist: %1.8f",
			   euclideanDist(thisArc->points.back(), newTracks[pointIndex]));
		}
	    }
	  else
	    {
	      //Not found, mark as inactive
	      thisArc->active=false;
	    }
	  pointIndex++;
	}

    }

    ROS_INFO("Found %d good correspondences", activePoints);


    //Do we need more tracks?
    int addedPoints = 0;
    if (activePoints < FeatureTracker::MIN_TRACKS)
      {
	vector<cv::Point2f> trackedPoints;

	getFeatures(graySrc, trackedPoints);
	
	int pointIndex = 0;
	float closeEps = 1.0f;
	for (int i=0; i<trackedPoints.size(); i++)
	  {
	    //Check to see if we're already tracking it
	    //The distance here should be essentially zero, since the feature detector was run
	    //on this image and the arc was already updated
	    
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

    //ROS_INFO("Added %d new points to track", addedPoints);
    //prepare the display
    dest_ptr->image = src_ptr->image.clone();
    cv::RNG rng(12345);
    //Draw all of the tracked features rng.uniform(0,255)
    cv::Scalar featColor = cv::Scalar(0, 0, 255);
    int arcsDrawn = 0;

    vector<cv::Point2f> featurePoints;

    getFeatures(graySrc, featurePoints);
    int count = 0;
    for (int i=0; i<  featurePoints.size(); i++)
      {
	//ROS_INFO("Drawing point %d",i++);
	//if (status[i] == 1) //the point was tracked from the past image...
	  {
	    cv::circle(dest_ptr->image, featurePoints[i], 3, featColor, -1, 8);
	  }
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
    dest_ptr->encoding = sensor_msgs::image_encodings::BGR8;
    std_msgs::Header hdr;
    hdr.seq = frameCount; 
    hdr.stamp = ros::Time::now();
    dest_ptr->header = hdr;
    image_pub_.publish(dest_ptr->toImageMsg());
    
    publishArcs(640,480);
    frameCount++;
    //Save the current image as the past
    graySrc.copyTo(prevGray);

  } catch(cv_bridge::Exception & e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
}

void FeatureTracker::publishArcs(uint16_t width, uint16_t height)
{
  int minSize = 10;
  
  cv::Mat outImage = cv::Mat(height, width, CV_8UC3, cv::Scalar(0));
  cv::RNG rng(12345);
  
  vector<feature_tracker::Arc> arcPoints;
  
  //Process the arc list and return a set of ordered points
  for (shared_ptr<ArcInfo> thisArc : arcs)
    {
      if (thisArc->points.size() < minSize)
	continue;
      Arc newArc;
      newArc.id = thisArc->id;
      for (cv::Point2f thisPoint : thisArc->points)
	{
	  newArc.x.push_back(thisPoint.x);
	  newArc.y.push_back(thisPoint.y);
	}
      arcPoints.push_back(newArc);
    }

  
  for (feature_tracker::Arc thisArc : arcPoints)
    {

      cv::Scalar thisColor = cv::Scalar(rng.uniform(0,255),rng.uniform(0,255),rng.uniform(0,255));
      for (int i=1; i<thisArc.x.size(); i++)
	{
	  cv::line(outImage, cv::Point(thisArc.x[i-1], thisArc.y[i-1]),
		   cv::Point(thisArc.x[i], thisArc.y[i]),
		   thisColor);

	}

    }

  //Convert to image message from cv::Mat and publish
  std_msgs::Header hdr;
  hdr.seq = frameCount; 
  hdr.stamp = ros::Time::now();
  cv_bridge::CvImage dest(hdr, sensor_msgs::image_encodings::RGB8, outImage);

  arc_pub_.publish(dest.toImageMsg());
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
