#ifndef VISIONBRAIN_HPP
#define VISIONBRAIN_HPP

#include <OffsetCalculator.h>
#include <StandAngleCalculator.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64MultiArray.h>

#include <opencv2/opencv.hpp>

class VisionBrain {
   public:
    VisionBrain();
    ~VisionBrain();
    void executeTasks();

   private:
    image_transport::Subscriber imageSub;
    void imageRecievedCallback(const sensor_msgs::ImageConstPtr& msg);

    ros::Subscriber laserScanSub;
    void scanRecievedCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

    ros::Publisher publisher;

    cv::Mat curFrame;
    sensor_msgs::LaserScan curScan;

    cv::String windowName;

    std_msgs::Float64MultiArray constructMessage(StandAngle stand, double offset);
};

#endif