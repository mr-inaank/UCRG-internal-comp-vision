// starter of code to image detect mandarins
#include <PoolDetector.h>

#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;

#define MAX_HEIGHT 800

Mat PoolDetector::getPoolMask(Mat im) {
    // Convert the images to HSV channels to filter by colour
    Mat im_HSV;
    cvtColor(im, im_HSV, COLOR_BGR2HSV);

    // add a colour masks to only include certain hue range
    Scalar lower_mask, upper_mask;
    lower_mask = Scalar(105, 85, 75);
    upper_mask = Scalar(110, 255, 255);

    Mat mask;
    inRange(im_HSV, lower_mask, upper_mask, mask);  // using hsv ranges

    Mat kernel = getStructuringElement(MORPH_RECT, Size(7, 7));
    morphologyEx(mask, mask, MORPH_OPEN, kernel);
    morphologyEx(mask, mask, MORPH_OPEN, kernel);
    morphologyEx(mask, mask, MORPH_CLOSE, kernel);

    return mask;
}

std::vector<Point> PoolDetector::extractPoolContour(Mat mask) {
    std::vector<std::vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    std::vector<Point> pool;

    if (contours.size() <= 0) {
        return pool;
    }

    // If there are too many contours, we select largest one.
    auto maxArea = 0;
    auto index = 0;
    for (auto i = 0; i < contours.size(); i++) {
        auto area = contourArea(contours[i]);
        if (area > maxArea) {
            maxArea = area;
            index = i;
        }
    }

    if (maxArea > 3000) {
        pool = contours[index];
    }

    return pool;
}

Point PoolDetector::getPoolCenter(std::vector<Point> pool) {
    auto M = moments(pool);
    auto cX = (int)(M.m10 / M.m00);
    auto cY = (int)(M.m01 / M.m00);

    Point result{ cX, cY };
    // ROS_INFO("Pool Center: (%d, %d)", cX, cY);
    return result;
}

std::vector<Point> PoolDetector::getPoolOffset(Mat im) {
    while (im.rows > MAX_HEIGHT) {
        resize(im, im, Size(), 0.5, 0.5, INTER_LINEAR);
    }
    Mat mask = getPoolMask(im);

    auto pool = extractPoolContour(mask);
    if (pool.size() <= 0) {
        return std::vector<Point>();
    }

    drawContours(im, std::vector<std::vector<Point>>{pool}, -1, Scalar(30, 70, 250), 2);

    // auto perimeter = arcLength(contours[index], true);
    // auto area = contourArea(pool);
    // auto circularity = 4 * CV_PI * area / (perimeter * perimeter);
    // std::cout << circularity << ", " << area << std::endl;

    auto poolCenter = getPoolCenter(pool);

    circle(im, poolCenter, 5, Scalar(0, 255, 0), 2);

    return std::vector<Point>{Point((int)im.cols / 2, (int)im.rows / 2) - poolCenter};
}
