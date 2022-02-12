#ifndef POOLDETECTOR_HPP
#define POOLDETECTOR_HPP

// #include <image_transport/image_transport.h>
// #include <ros/ros.h>
// #include <std_msgs/Bool.h>

#include <opencv2/opencv.hpp>

class PoolDetector {
   public:
    static cv::Point getPoolOffset(cv::Mat im);

   private:
    static cv::Point getPoolCenter(std::vector<cv::Point> pool);
    static std::vector<cv::Point> extractPoolContour(cv::Mat mask);
    static cv::Mat getPoolMask(cv::Mat im);
};

#endif