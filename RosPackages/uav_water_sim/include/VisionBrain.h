#ifndef VISIONBRAIN_HPP
#define VISIONBRAIN_HPP

#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>

#include <opencv2/opencv.hpp>

class VisionBrain {
public:
    VisionBrain();
    ~VisionBrain();
    bool executeTasks();

private:
    image_transport::Subscriber imageSub;
    void imageRecievedCallback(const sensor_msgs::ImageConstPtr& msg);

    ros::Publisher movePub;
    void VisionBrain::move(int forwardBackward, int leftRight, int upDown, int yawLeftRight, int actuatorOpen);

    cv::Mat curFrame;

    cv::String windowName;

    void printInstruction(cv::Point locationOffset);

    int taskNumber = 0;
    int counter = 0;

    void takeoff();
    void findPool();
    void descendAboveZone();
    void waitForWaterCollection();
    void findCheckerBoard();
    void returnToHome();
};

#endif