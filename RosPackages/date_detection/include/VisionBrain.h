#ifndef VISIONBRAIN_HPP
#define VISIONBRAIN_HPP

#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>

#include <opencv2/opencv.hpp>

class VisionBrain {
public:
    VisionBrain();
    ~VisionBrain();
    bool executeTasks();

private:
    image_transport::Subscriber imageSub;
    void imageRecievedCallback(const sensor_msgs::ImageConstPtr& msg);

    ros::Subscriber dateGrabberSub;
    void dateGrabberIsActiveCallback(const std_msgs::Float64MultiArray::ConstPtr& pumpFinished);
    bool isDateGrabberActive = false;

    ros::Publisher dateGrabberPub;
    void activateDateGrabber();

    cv::Mat curFrame;
    sensor_msgs::LaserScan curScan;

    cv::String windowName;

    void printInstruction(cv::Point locationOffset);

    int taskNumber = 0;
    int counter;
    int numDates = 0;

    void moveToDateArea();
    void collectDates();
    void waitForCollectingDate();
    void moveBackUp();
    void moveToNextDateLine();
    void moveToStart();
};

#endif
