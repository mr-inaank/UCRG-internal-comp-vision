#ifndef OFFSETCALCULATOR_HPP
#define OFFSETCALCULATOR_HPP

#include <opencv2/opencv.hpp>

class OffsetCalculator {
   public:
    static double calcOffset(cv::Mat im);

   private:
    static cv::Mat extractSegments(cv::Mat im);
    static cv::Point2f findRedDot(cv::Mat im);
};

#endif