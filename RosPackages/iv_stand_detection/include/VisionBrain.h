#ifndef VISIONBRAIN_HPP
#define VISIONBRAIN_HPP

#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <opencv2/opencv.hpp>

class VisionBrain {
   public:
    VisionBrain();
    ~VisionBrain();

   private:
    image_transport::Subscriber imageSub;
    void imageRecievedCallback(const sensor_msgs::ImageConstPtr& msg);

    ros::Subscriber laserScanSub;
    void scanRecievedCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

    cv::Mat curFrame;
    std::vector<float> curRanges;

    cv::String windowName;
};

#endif