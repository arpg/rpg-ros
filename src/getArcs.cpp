/* Call the GetArcs service and display some arcs */
#include <ros/ros.h>
#include <feature_tracker/GetArcs.h>
#include <feature_tracker/Arc.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <cstdlib>

void drawArcs(std::vector<feature_tracker::Arc> &arcs, uint16_t width, uint16_t height)
{
  cv::Mat outImage = cv::Mat(height, width, CV_8UC3);
  cv::RNG rng(12345);
  for (feature_tracker::Arc thisArc : arcs)
    {
      //cv::circle(outImage,
      cv::Scalar thisColor = cv::Scalar(rng.uniform(0,255),rng.uniform(0,255),rng.uniform(0,255));
      for (int i=1; i<thisArc.x.size(); i++)
	{
	  cv::line(outImage, cv::Point(thisArc.x[i-1], thisArc.y[i-1]),
		   cv::Point(thisArc.x[i], thisArc.y[i]),
		   thisColor);
	   
	  //outImage.at(thisArc.x[i], thisArc.y[i]) = cv::Scalar(255,0,0);
	}

    }
  cv::imshow("Arcs", outImage);
  cv::waitKey(0);
  cv::imwrite("output.jpg", outImage);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "getArcs_client");
  if (argc != 2)
  {
    ROS_INFO("usage: getArcs <min length>");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<feature_tracker::GetArcs>("GetArcs");
  feature_tracker::GetArcs srv;
  srv.request.minSize = atoll(argv[1]);
  if (client.call(srv))
  {
    ROS_INFO("Got %d arcs", srv.response.arcs.size());
    drawArcs(srv.response.arcs, srv.response.imageWidth, srv.response.imageHeight);
  }
  else
  {
    ROS_ERROR("Failed to call service GetArcs");
    return 1;
  }

  return 0;
}
