#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Header.h>

#include <opencv2/highgui/highgui.hpp>

using namespace cv;
Mat curFrame;
void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        curFrame = cv_bridge::toCvShare(msg, "bgr8")->image;
        imshow("view", curFrame);
        waitKey(30);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void depthCloudCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    Mat img{480, 640, CV_8UC3, Scalar(0, 0, 0)};
    Mat img2 = curFrame.clone();

    auto maxTheta = msg->angle_min + 639 * msg->angle_increment;
    for (auto i = 0; i < msg->ranges.size(); i++) {
        auto theta = msg->angle_min + i * msg->angle_increment;
        auto r = msg->ranges.at(i);

        auto x = 640 - 640 * pow((theta + msg->angle_max) / (maxTheta - msg->angle_min), 0.98);
        auto y = -r * 100 * cos(theta) + 200;
        ROS_INFO("%f, %f, %f", x, y, r);
        if (r >= 0) circle(img2, Point(x, 240), 3, Scalar(0, r / 3 * 255, 0), 1);
    }
    ROS_INFO("=================");

    imshow("scan", img2);
    waitKey(30);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    namedWindow("view");
    namedWindow("scan");

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/simon/camera_base_mount/simon/image_raw", 1, imageCallback);

    ros::Subscriber sub2 = nh.subscribe("/scan", 1, depthCloudCallback);

    ROS_INFO("%s", sub.getTopic().c_str());
    ros::spin();
    destroyWindow("view");
    destroyWindow("scan");
}