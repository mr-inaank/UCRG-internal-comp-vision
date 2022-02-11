#ifndef DATEDETECTOR_HPP
#define DATEDETECTOR_HPP

#include <opencv2/opencv.hpp>


class DateDetector {
public:
    static std::vector<cv::Point> findDates(cv::Mat im);

private:
    static cv::Mat getDateMask(cv::Mat im);
};


#endif  