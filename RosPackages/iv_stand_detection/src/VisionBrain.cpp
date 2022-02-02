#include <OffsetCalculator.h>
#include <VisionBrain.h>
#include <cv_bridge/cv_bridge.h>

VisionBrain::VisionBrain() {
    ros::NodeHandle nh;

    windowName = cv::String("view");
    cv::namedWindow(windowName);
    // cv::namedWindow("mask");

    image_transport::ImageTransport it{nh};
    imageSub = it.subscribe("/simon/camera_base_mount/simon/image_raw", 1, &VisionBrain::imageRecievedCallback, this);

    laserScanSub = nh.subscribe("/scan", 1, &VisionBrain::scanRecievedCallback, this);
}

VisionBrain::~VisionBrain() {
    cv::destroyWindow(windowName);
    // cv::destroyWindow("mask");
}

void VisionBrain::imageRecievedCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        curFrame = cv_bridge::toCvShare(msg, "bgr8")->image;

        auto offset = OffsetCalculator::calcOffset(curFrame);
        ROS_INFO("Red dot distance from center: %f", offset);

        // cv::imshow(windowName, curFrame);
        // cv::waitKey(25);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void VisionBrain::scanRecievedCallback(const sensor_msgs::LaserScan::ConstPtr& scan) { curRanges = scan->ranges; }