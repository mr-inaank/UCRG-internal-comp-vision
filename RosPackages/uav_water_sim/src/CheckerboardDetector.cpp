// starter of code to image detect mandarins
#include <CheckerboardDetector.h>
#include <opencv2/opencv.hpp>

using namespace cv;

#define MAX_HEIGHT 800

Mat CheckerboardDetector::preprocessImage(Mat im) {
    // Convert the images to HSV channels to filter by colour
    Mat im_HSV, gray;
    cvtColor(im, im_HSV, COLOR_BGR2HSV);
    cvtColor(im, gray, COLOR_BGR2GRAY);

    Mat mask;
    cornerHarris(gray, mask, 3, 5, 0.1);
    mask.convertTo(mask, CV_8UC1, 255, 0);
    threshold(mask, mask, 0, 255, THRESH_BINARY);

    Mat kernel = getStructuringElement(MORPH_RECT, Size(35, 35));
    morphologyEx(mask, mask, MORPH_CLOSE, kernel);

    kernel = getStructuringElement(MORPH_RECT, Size(27, 27));
    morphologyEx(mask, mask, MORPH_OPEN, kernel);

    return mask;

    // add a colour masks to only include certain hue range
    // Scalar lower_mask, upper_mask;
    // lower_mask = Scalar(0, 0, 200);
    // upper_mask = Scalar(255, 15, 215);

    // Mat mask;
    // inRange(im_HSV, lower_mask, upper_mask, mask);

    // kernel = getStructuringElement(MORPH_RECT, Size(15, 15));
    // morphologyEx(mask, mask, MORPH_CLOSE, kernel);

    // kernel = getStructuringElement(MORPH_RECT, Size(27, 27));
    // morphologyEx(mask, mask, MORPH_OPEN, kernel);

    // return mask;
}

std::vector<Point> CheckerboardDetector::getCheckerboardLocation(Mat im) {
    Mat mask = preprocessImage(im);

    std::vector<Point> points;
    findNonZero(mask, points);

    std::vector<std::vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    std::vector<Point> checkerBoard;

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

    if (maxArea < 3500) {
        return std::vector<Point>();
    }

    auto board = contours[index];

    Rect rect = boundingRect(board);
    rectangle(im, rect, Scalar(0, 0, 255), 3);

    Point center = (rect.br() + rect.tl()) * 0.5;
    circle(im, center, 7, Scalar(0, 255, 0), 3);

    return std::vector<Point>{Point((int)im.cols / 2, (int)im.rows / 2) - center};
}
