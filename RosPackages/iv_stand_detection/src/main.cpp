#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

cv::Mat curFrame;

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
        cv::waitKey(0);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void scanCallback(const sensor_msgs::LaserScanCal::ConstPtr& scan) {
    try {
        std::cout << scan.intensities.length() << std::endl;
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "offsetCalculator");
    ros::NodeHandle nh;

    cv::namedWindow("view");
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/simon/camera_base_mount/simon/image_raw", 1, imageCallback);

    ros::Subscriber sub = nh.subscribe("/scan", 1000, scanCallback);


    ROS_INFO("%s", sub.getTopic().c_str());
    ros::spin();
    cv::destroyWindow("view");
}