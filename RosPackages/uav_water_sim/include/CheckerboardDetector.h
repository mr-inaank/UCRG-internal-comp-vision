#ifndef CHECKERBOARDDETECTOR_HPP
#define CHECKERBOARDDETECTOR_HPP

// #include <image_transport/image_transport.h>
// #include <ros/ros.h>
// #include <std_msgs/Bool.h>

#include <opencv2/opencv.hpp>

class CheckerboardDetector {
   public:
    static std::vector<cv::Point> getCheckerboardLocation(cv::Mat im);

   private:
    static cv::Mat preprocessImage(cv::Mat im);
};

#endif