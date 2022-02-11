#ifndef VISIONBRAIN_HPP
#define VISIONBRAIN_HPP

#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

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
    void dateGrabberIsActiveCallback(const std_msgs::Bool::ConstPtr& pumpFinished);
    bool isDateGrabberActive = false;

    ros::Publisher dateGrabberPub;
    void activateDateGrabber();

    cv::Mat curFrame;

    cv::String windowName;

    void printInstruction(cv::Point locationOffset);

    int taskNumber = 0;
    int counter;

    void moveToDateArea();
    void collectDates();
    void waitForCollectingDate();
    void moveBackUp();
    void moveToNextDateLine();
    void moveToStart();
};

#endif