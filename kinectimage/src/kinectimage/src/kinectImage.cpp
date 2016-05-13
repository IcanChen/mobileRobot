/**
*@file kinectImage.cpp
*@brief  Simultaneous acquisition  depth image and rgb image of kinect
*使用多线程获取和处理kinect数据，一个线程订阅接收kinect RGB图像和深度图像，一个线程用于处理RGB图像和深度图像。
*
*@author 陈明建,chengmingjian@cvte.com
*@version 1.0
*@date  2016.05.12
**/

#include <ros/ros.h>
#include <string.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/thread.hpp>


boost::mutex imageMutex;

static const std::string RGB_WINDOW = "RGB Image window";
static const std::string DEPTH_WINDOW = "Depth Image window";

cv::Mat rawDepthImage;
cv::Mat rawRgbImage;
cv::Mat newestDepthImage;
cv::Mat newestRgbImage;

class SubscriberKinectImage
{
private:
  ros::NodeHandle nih_;
  image_transport::ImageTransport imageit_;
  image_transport::Subscriber image_sub_;

  ros::NodeHandle ndh_;
  image_transport::ImageTransport depthit_;
  image_transport::Subscriber depth_sub_;
  
public:
  SubscriberKinectImage():imageit_(nih_),depthit_(ndh_)
  {
    image_sub_ = imageit_.subscribe("/camera/rgb/image_raw", 1000, 
    &SubscriberKinectImage::rgbCb, this);

    depth_sub_ = depthit_.subscribe("/camera/depth_registered/image_raw", 1000, 
    &SubscriberKinectImage::depthCb, this);
  }

  ~SubscriberKinectImage()
  {
  }
  
  void depthCb(const sensor_msgs::ImageConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(imageMutex);
    try
    {
       rawDepthImage = cv_bridge::cvtColorForDisplay(cv_bridge::toCvShare(msg),"",false)->image;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
     return;
    }
  }

  void rgbCb(const sensor_msgs::ImageConstPtr& msg)
  {  
    boost::mutex::scoped_lock lock(imageMutex);
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); 
      rawRgbImage = cv_ptr->image;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
     return;
    } 
  }

};


void updateImage()
{
  boost::mutex::scoped_lock lock(imageMutex);
  rawRgbImage.copyTo(newestRgbImage);
  rawDepthImage.copyTo(newestDepthImage);
}

void handleImageThread()
{
  cv::namedWindow(RGB_WINDOW);
  cv::namedWindow(DEPTH_WINDOW);
  
  while (1)
  {
    updateImage();
     
    if (!newestDepthImage.empty())
    {
      cv::imshow(RGB_WINDOW, newestDepthImage);
      cv::waitKey(3);
    }

    if (!newestRgbImage.empty())
    {
      cv::imshow(DEPTH_WINDOW, newestRgbImage);
      cv::waitKey(3);
    }
  }
}

int main(int argc, char** argv)
{
  boost::thread handleImage(&handleImageThread);

  ros::init(argc, argv, "kinectImage");
  
  SubscriberKinectImage ic;

  ros::spin();
  return 0;
}