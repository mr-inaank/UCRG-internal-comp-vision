#ifndef CHECKERBOARD_DETECOR_UGV_HPP
#define CHECKERBOARD_DETECOR_UGV_HPP

#include <opencv2/opencv.hpp>

const cv::Point NULL_POINT{ -1,-1 };

class CheckerBoardDetector {
public:
    static cv::Point getCheckerboardCenter(cv::Mat im);


private:
    static cv::Mat preprocessImage(cv::Mat im);

};


#endif