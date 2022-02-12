// // starter of code to image detect mandarins
// #include <CheckerboardDetector.h>
// #include <opencv2/opencv.hpp>

// using namespace cv;

// #define MAX_HEIGHT 800
// cv::Point prevCenter = NULL_POINT;


// Mat CheckerboardDetector::preprocessImage(Mat im) {
//     // Convert the images to HSV channels to filter by colour
//     Mat im_HSV, gray;
//     cvtColor(im, im_HSV, COLOR_BGR2HSV);
//     cvtColor(im, gray, COLOR_BGR2GRAY);

//     // add a colour masks to only include certain hue range
//     Scalar lower_mask, upper_mask;
//     lower_mask = Scalar(0, 0, 144);
//     upper_mask = Scalar(255, 255, 255);
//     // lower_mask = Scalar(0, 0, 0);
//     // upper_mask = Scalar(255, 11, 255);

//     Mat mask;
//     inRange(im_HSV, lower_mask, upper_mask, mask); // using hsv ranges

//     Mat kernel = getStructuringElement(MORPH_RECT, Size(11, 11));
//     morphologyEx(mask, mask, MORPH_OPEN, kernel);

//     return mask;
// }

// Point CheckerboardDetector::getCheckerboardLocation(Mat im) {
//     Mat mask = preprocessImage(im);

//     Mat result1;
//     cornerHarris(mask, result1, 3, 5, 0.1);
//     result1.convertTo(result1, CV_8UC1, 255, 0);

//     Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
//     morphologyEx(result1, result1, MORPH_CLOSE, kernel);
//     kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
//     // morphologyEx(result1, result1, MORPH_ERODE, kernel);

//     imshow("'mask'", mask);
//     imshow("'resu'", result1);

//     std::vector<Point> points;
//     findNonZero(result1, points);

//     Rect rect = boundingRect(points);
//     rectangle(im, rect, Scalar(0, 0, 255), 1);
//     if (rect.area() > 10000) {
//         rectangle(im, rect, Scalar(0, 0, 255), 3);

//         Point center = (rect.br() + rect.tl()) * 0.5;
//         circle(im, center, 5, Scalar(0, 255, 0), 1);
//         auto lr = 0.3;
//         if (prevCenter != NULL_POINT) {
//             center = center * lr + prevCenter * (1 - lr);
//         }
//         circle(im, center, 7, Scalar(0, 255, 0), 3);
//         prevCenter = center;
//         rectangle(im, Point((int)im.cols / 2 - 100, (int)im.rows / 2 - 100), Point((int)im.cols / 2 + 100, (int)im.rows / 2 + 100), Scalar(255, 255, 0), 3);
//         return center;
//     } else {
//         prevCenter = NULL_POINT;
//         return NULL_POINT;
//     }
// }
// starter of code to image detect mandarins
#include <CheckerboardDetector.h>
#include <opencv2/opencv.hpp>

using namespace cv;

#define MAX_HEIGHT 800
cv::Point prevCenter = NULL_POINT;


Mat CheckerboardDetector::preprocessImage(Mat im) {
    // Convert the images to HSV channels to filter by colour
    Mat im_HSV;
    cvtColor(im, im_HSV, COLOR_BGR2HSV);

    // add a colour masks to only icnclude certain hue range
    Scalar lower_bound, upper_bound;
    lower_bound = Scalar(0, 0, 150);
    upper_bound = Scalar(255, 255, 255);
    Mat mask;
    inRange(im_HSV, lower_bound, upper_bound, mask); // using hsv ranges

    Mat kernel = getStructuringElement(MORPH_RECT, Size(15, 15));
    morphologyEx(mask, mask, MORPH_ERODE, kernel);

    return mask;
}

Mat CheckerboardDetector::getCornersImage(Mat mask) {
    std::vector<std::vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    Mat result = Mat(mask.rows, mask.cols, CV_8UC1, Scalar(0));
    for (auto i = 0; i < contours.size(); i++) {
        auto area = contourArea(contours[i], false);

        if (area > 4000 || area < 100) {
            continue;
        }

        auto rect = minAreaRect(contours[i]);
        auto aspectRatio = 0;
        if (rect.size.width != 0 && rect.size.height != 0) {
            aspectRatio = max(rect.size.width, rect.size.height) / min(rect.size.width, rect.size.height);
        }

        if (aspectRatio < 1.2) {
            circle(result, rect.center, 20, Scalar(255), -1);
        }
    }

    Mat kernel = getStructuringElement(MORPH_RECT, Size(31, 31));
    morphologyEx(result, result, MORPH_CLOSE, kernel);
    morphologyEx(result, result, MORPH_OPEN, kernel);

    return result;
}

Point CheckerboardDetector::getCheckerboardLocation(Mat im) {
    Mat mask = preprocessImage(im);
    Mat result = getCornersImage(mask);

    std::vector<std::vector<Point>> contours;
    findContours(result, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    auto index = -1;
    auto maxArea = 0;
    for (auto i = 0; i < contours.size(); i++) {
        auto area = contourArea(contours[i]);
        if (area < 3000 || area < maxArea) {
            continue;
        }

        maxArea = area;
        index = i;
    }

    if (index >= 0) {
        auto rect = boundingRect(contours[index]);
        rectangle(im, rect, Scalar(0, 0, 255), 3);
        imshow("imas", im);

        return (rect.br() + rect.tl()) * 0.5;

    } else {
        return NULL_POINT;
    }
}
