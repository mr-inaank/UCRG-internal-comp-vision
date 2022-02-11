// starter of code to image detect mandarins
#include <CheckerboardDetector.h>
#include <opencv2/opencv.hpp>

using namespace cv;

#define MAX_HEIGHT 800
cv::Point prevCenter = NULL_POINT;


Mat CheckerboardDetector::preprocessImage(Mat im) {
    // Convert the images to HSV channels to filter by colour
    Mat im_HSV, gray;
    cvtColor(im, im_HSV, COLOR_BGR2HSV);
    cvtColor(im, gray, COLOR_BGR2GRAY);

    // add a colour masks to only include certain hue range
    Scalar lower_mask, upper_mask;
    Scalar lower_mask2, upper_mask2;
    lower_mask = Scalar(0, 0, 0);
    upper_mask = Scalar(255, 11, 255);
    lower_mask2 = Scalar(0, 4, 144);
    upper_mask2 = Scalar(255, 45, 255);

    Mat mask1, mask2;
    inRange(im_HSV, lower_mask, upper_mask, mask1); // using hsv ranges
    inRange(im_HSV, lower_mask2, upper_mask2, mask2); // using hsv ranges

    Mat mask = mask1.clone();

    Mat kernel = getStructuringElement(MORPH_RECT, Size(11, 11));
    morphologyEx(mask, mask, MORPH_OPEN, kernel);

    return mask;
}

Point CheckerboardDetector::getCheckerboardLocation(Mat im) {
    Mat mask = preprocessImage(im);

    Mat result1;
    cornerHarris(mask, result1, 3, 5, 0.1);
    result1.convertTo(result1, CV_8UC1, 255, 0);

    Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(result1, result1, MORPH_CLOSE, kernel);
    kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    morphologyEx(result1, result1, MORPH_ERODE, kernel);

    std::vector<Point> points;
    findNonZero(result1, points);

    Rect rect = boundingRect(points);
    rectangle(im, rect, Scalar(0, 0, 255), 1);
    if (rect.area() > 10000) {
        rectangle(im, rect, Scalar(0, 0, 255), 3);

        Point center = (rect.br() + rect.tl()) * 0.5;
        circle(im, center, 5, Scalar(0, 255, 0), 1);
        auto lr = 0.3;
        if (prevCenter != NULL_POINT) {
            center = center * lr + prevCenter * (1 - lr);
        }
        circle(im, center, 7, Scalar(0, 255, 0), 3);
        prevCenter = center;
	rectangle(im, Point((int) im.cols/2 -100, (int) im.rows/2- 100), Point((int) im.cols/2 + 100, (int) im.rows/2 + 100), 3);

        return center;
    } else {
        prevCenter = NULL_POINT;
        return NULL_POINT;
    }

}
