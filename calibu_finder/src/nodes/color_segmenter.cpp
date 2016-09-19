
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>


#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>

/*

Code from :
http://www.learnopencv.com/blob-detection-using-opencv-python-c/
http://opencv-srf.blogspot.com/2010/09/object-detection-using-color-seperation.html
 */



namespace calibu_finder
{

class ColorSegmenter
{
public:
  ColorSegmenter();

  void process(const sensor_msgs::ImageConstPtr& msg);
private:
  ros::Publisher image_pub_;
  ros::NodeHandle n_;
  image_transport::ImageTransport it;
  image_transport::Subscriber image_sub;
  // Params
  int hue_;
  int hue_window_;
  
  uint8_t display_;
};

}

using namespace calibu_finder;


ColorSegmenter::ColorSegmenter() :
  it(n_)
{
  image_pub_ = n_.advertise<sensor_msgs::Image>("masked", 1);
  image_sub = it.subscribe("/image", 1, &ColorSegmenter::process, this);

  ros::NodeHandle local_ns_("~");

  hue_ = local_ns_.param("hue", 20);
  hue_window_ = local_ns_.param("hue_window", 5);
  display_ = local_ns_.param("display", 0);
  
  ROS_INFO("[hue]: %i", hue_);
  ROS_INFO("[display]: %i", display_);
  if (display_)
    {
      ROS_INFO("Displaying results");
      cv::namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
      cv::createTrackbar("Hue", "Control", &hue_, 179); //Hue (0 - 179)
      cv::createTrackbar("Hue Window", "Control", &hue_window_, 10); //Hue (0 - 179)
    }
}

void ColorSegmenter::process(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr src_ptr;
  cv_bridge::CvImagePtr hsv_ptr(new cv_bridge::CvImage);
  cv_bridge::CvImagePtr thold_ptr(new cv_bridge::CvImage);
  cv_bridge::CvImagePtr rgb_t_ptr(new cv_bridge::CvImage);
  cv_bridge::CvImagePtr dest_ptr(new cv_bridge::CvImage);
  
  try {
    //Find the color blob, use as a mask over the source color image for the result
    src_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    
    cv::cvtColor(src_ptr->image, hsv_ptr->image, cv::COLOR_BGR2HSV);
    cv::inRange(hsv_ptr->image, cv::Scalar(hue_-hue_window_, 0,0), cv::Scalar(hue_+hue_window_, 255, 255), thold_ptr->image);
    //morphological opening (remove small objects from the foreground)
    cv::erode(thold_ptr->image, thold_ptr->image, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    cv::erode(thold_ptr->image, thold_ptr->image, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    cv::dilate(thold_ptr->image, thold_ptr->image, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 

    //morphological closing (fill small holes in the foreground)
    cv::dilate(thold_ptr->image, thold_ptr->image, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    cv::dilate(thold_ptr->image, thold_ptr->image, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(8, 8)) );
    //cv::erode(thold_ptr->image, thold_ptr->image, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 

    if (display_)
      {
	cv::imshow("THold", thold_ptr->image);

      }
    // Blob detect on the thresholded image to get center mass

    cv::SimpleBlobDetector blob_detector;
    std::vector<cv::KeyPoint> keypoints;
    blob_detector.detect(thold_ptr->image, keypoints);

    for (int i=0; i<keypoints.size(); i++)
      {
	printf("blob[%d]:  x:%f y:%f\n", i, keypoints[i].pt.x, keypoints[i].pt.y); 
      }

    cv::cvtColor(thold_ptr->image, rgb_t_ptr->image, cv::COLOR_GRAY2BGR);

    //Apply the mask
    cv::bitwise_and(rgb_t_ptr->image, src_ptr->image, dest_ptr->image);

    if (display_)
      {
	cv::imshow("Masked", dest_ptr->image);
      }
    if (display_)
      cv::waitKey(1);
    // Send the annotated image over ROS
    dest_ptr->encoding = sensor_msgs::image_encodings::BGR8;
    
    image_pub_.publish(dest_ptr->toImageMsg());
  } catch(cv_bridge::Exception & e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "color_segmenter");

  //ROS_INFO("Starting color_segmenter");
  ColorSegmenter seg;


  ros::spin();
  return 0;
}
