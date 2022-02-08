#include <iostream>
#include <opencv2/highgui.hpp>

#include <DateDetector.h>

using namespace cv;

#define MAX_HEIGHT 800

const Point NULL_POINT{-1, -1};

Mat DateDetector::getDateMask(Mat im) {
    Mat gray;
    cvtColor(im, gray, COLOR_BGR2GRAY);

    Mat mask;
    threshold(gray, mask, 30, 255, THRESH_BINARY_INV);

    Mat cleaningMask = Mat(mask.rows, mask.cols, CV_8UC1, 255);
    // cleaningMask(Rect(Point(381, 367), Point(501, im.rows))) = 0;
    cleaningMask(Rect(Point(236, im.rows), Point(371, 367))) = 0;
    bitwise_and(mask, cleaningMask, mask);

    Mat kernel;
    kernel = getStructuringElement(MORPH_RECT, Size(11, 11));
    morphologyEx(mask, mask, MORPH_CLOSE, kernel);
    morphologyEx(mask, mask, MORPH_OPEN, kernel);

    return mask;
}

Point DateDetector::processFrame(Mat im) {

    while (im.rows > MAX_HEIGHT) {
        resize(im, im, Size(), 0.5, 0.5, INTER_LINEAR);
    }

    Mat mask = getDateMask(im);

    std::vector<std::vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    int index = -1;
    double maxArea = 0;
    for (auto i = 0; i < contours.size(); i++) {
        auto area = contourArea(contours[i]);
        if (maxArea < area) {
            index = i;
            maxArea = area;
        }
    }

    if (index < 0) {
        return NULL_POINT;
    }

    if (maxArea < 300) {
        drawContours(im, contours, index, Scalar(255, 0, 0), 1);
        return NULL_POINT;
    } else {
        drawContours(im, contours, index, Scalar(0, 255, 0), 1);
    }

    Rect rect = boundingRect(contours[index]);
    rectangle(im, rect, Scalar(255, 0, 0), 2);
    Point center = (rect.br() + rect.tl()) * 0.5;

    return center;
}

int main() {
    return 0;
}