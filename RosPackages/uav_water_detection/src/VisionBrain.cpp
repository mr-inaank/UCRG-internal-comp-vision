#include <PoolDetector.h>
#include <VisionBrain.h>
#include <cv_bridge/cv_bridge.h>

VisionBrain::VisionBrain() {
    ros::NodeHandle nh;

    windowName = cv::String("view");
    cv::namedWindow(windowName);

    image_transport::ImageTransport it{nh};
    imageSub = it.subscribe("/simon/camera_base_mount/simon/image_raw", 1, &VisionBrain::imageRecievedCallback, this);

    pumpPub = nh.advertise<std_msgs::Bool>("/uav/pump/activate", 1);
    pumpSub = nh.subscribe("/uav/pump/isActive", 1, &VisionBrain::pumpIsActiveCallback, this);
}

VisionBrain::~VisionBrain() { cv::destroyWindow(windowName); }

void VisionBrain::imageRecievedCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        curFrame = cv_bridge::toCvShare(msg, "bgr8")->image;

        // cv::imshow(windowName, curFrame);
        // cv::waitKey(25);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void VisionBrain::pumpIsActiveCallback(const std_msgs::Bool::ConstPtr& isActive) { isPumpActive = isActive->data; }

void VisionBrain::executeTasks() {
    if (curFrame.empty()) {
        ROS_ERROR("No image recieved");
        return;
    }

    switch (taskNumber) {
        case 0:
            counter = 0;
            takeoff();
        case 1:
            findPool();
    }

    activatePump(false);
}

void VisionBrain::activatePump(bool activate) {
    std_msgs::Bool result;
    result.data = activate;

    pumpPub.publish(result);
}

void VisionBrain::takeoff() {
    if (counter < 10) {
        ROS_INFO("Command: UP");
        counter++;
        return;
    }

    taskNumber++;
    ROS_INFO("Command: STOP");
}

void VisionBrain::findPool() {
    ROS_INFO("Command: LEFT");

    auto result = PoolDetector::getPoolLocation(curFrame);
    return;
}