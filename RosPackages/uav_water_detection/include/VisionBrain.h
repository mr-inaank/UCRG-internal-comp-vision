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

    ros::Subscriber pumpSub;
    void pumpIsActiveCallback(const std_msgs::Bool::ConstPtr& pumpFinished);
    bool isPumpActive;

    ros::Publisher pumpPub;
    void activatePump(bool activate);

    cv::Mat curFrame;

    cv::String windowName;

    void printInstruction(cv::Point locationOffset);

    int taskNumber = 1;
    int counter;

    void takeoff();
    void findPool();
    void descendAboveZone();
    void waitForWaterCollection();
    void findCheckerBoard();
    void returnToHome();
};

#endif