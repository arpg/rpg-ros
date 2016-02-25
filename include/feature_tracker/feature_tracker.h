#ifndef _FEATURE_TRACKER_H_
#define _FEATURE_TRACKER_H_

//Feature datatype definitions

#include <boost/thread/mutex.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <image_transport/image_transport.h>
#include <feature_tracker/RobustMatcher.h>
#include <feature_tracker/GetArcs.h>
#include <feature_tracker/ClearArcs.h>
//

typedef struct FrameInfo
{
  uint32_t seq;
  cv::Mat descriptors;
  std::vector<cv::KeyPoint> keypoints;
  std::vector<cv::DMatch> matches;
}
frameinfo_t;

typedef struct ArcInfo
{
  uint32_t id; //monotonic increasing ID, assigned on first match
  uint32_t firstSeen; //seq of first appearance
  std::vector<cv::KeyPoint> points;
  std::vector<int> matchIDs; //matches, starting from first matched frame. Used to correlate between frames
  bool active;
  bool touched;
  ArcInfo()
  {
    active = false;
    touched = false;
  }
}
  arcinfo_t;
#endif
