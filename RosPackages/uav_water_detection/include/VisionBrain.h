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
    void executeTasks();

   private:
    image_transport::Subscriber imageSub;
    void imageRecievedCallback(const sensor_msgs::ImageConstPtr& msg);

    ros::Subscriber pumpSub;
    void pumpIsActiveCallback(const std_msgs::Bool::ConstPtr& isActive);
    bool isPumpActive;

    ros::Publisher pumpPub;
    void activatePump(bool activate);

    cv::Mat curFrame;

    cv::String windowName;

    int taskNumber = 1;
    int counter;

    void takeoff();
    void findPool();
};

#endif