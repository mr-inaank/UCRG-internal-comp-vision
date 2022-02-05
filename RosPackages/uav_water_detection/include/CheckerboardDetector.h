#ifndef CHECKERBOARDDETECTOR_HPP
#define CHECKERBOARDDETECTOR_HPP

#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <opencv2/opencv.hpp>

const auto NULL_POINT = cv::Point(-1, -1);

class CheckerboardDetector {
   public:
    static cv::Point getCheckerboardLocation(cv::Mat im);

   private:
    static cv::Mat preprocessImage(cv::Mat im);
};

#endif