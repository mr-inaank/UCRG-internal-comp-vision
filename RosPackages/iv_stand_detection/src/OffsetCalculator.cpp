#include <OffsetCalculator.h>
#include <ros/ros.h>

using namespace cv;

double OffsetCalculator::calcOffset(Mat im) {
    Point2f result = findRedDot(im);
    if (result.x == -1 && result.y == -1) {
        return nan("");
    }

    auto centerLine = (static_cast<double>(im.cols)) / 2.0;
    auto offset = result.x - centerLine;

    return offset;
}

Mat OffsetCalculator::extractSegments(Mat im) {
    Mat im_HSV;
    cvtColor(im, im_HSV, COLOR_BGR2HSV);

    // Segment
    Scalar lower_range, upper_range;
    lower_range = Scalar(0, 180, 0);
    upper_range = Scalar(5, 255, 255);
    Mat mask;
    inRange(im_HSV, lower_range, upper_range, mask);  // using hsv ranges

    // Clean
    Mat kernel = Mat(10, 10, CV_8UC1, 1);
    morphologyEx(mask, mask, MORPH_CLOSE, kernel);

    return mask;
}

Point2f OffsetCalculator::findRedDot(Mat im) {
    Mat mask = extractSegments(im);

    SimpleBlobDetector::Params params;

    params.filterByArea = true;
    params.minArea = 7;

    params.filterByCircularity = false;
    params.minCircularity = 0.9;

    params.filterByConvexity = false;
    params.minConvexity = 0.75;

    params.filterByInertia = false;
    params.minInertiaRatio = 0.01;

    std::vector<KeyPoint> keypoints;
    Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);

    // change the image to black on white background
    // (just need to get rid of white points)
    bitwise_not(mask, mask);

    // Detect blobs
    detector->detect(mask, keypoints);

    // if (keypoints.size() > 0) {
    //     circle(im, keypoints[0].pt, 10, Scalar(0, 0, 255), 3);
    // }

    if (keypoints.size() != 1) {
        ROS_ERROR("Oh shit! Found %d keypoints.", (int)keypoints.size());
        return Point2f(-1, -1);
    }

    return keypoints[0].pt;
}