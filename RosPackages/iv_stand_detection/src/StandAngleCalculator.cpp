#include <StandAngleCalculator.h>
#include <ros/ros.h>

using namespace cv;

double findThirdSide(double a, double b, double C) { return sqrt(a * a + b * b - 2 * a * b * sin(C)); }

int getRangeIndex(int x, const sensor_msgs::LaserScan& msg) {
    auto maxTheta = msg.angle_min + 639 * msg.angle_increment;

    auto powerCorrection = 0.98;

    auto index = (pow(1-x/640.0, 1.0/0.98) * (maxTheta - msg.angle_min) - msg.angle_max - msg.angle_min) /
                 msg.angle_increment;

    index = max(0.0, min(639.0, floor(index)));
 
    return (int)index;
}

StandAngle StandAngleCalculator::calcStandAngle(Mat im, const sensor_msgs::LaserScan& scan) {
    HSVRange yellowrawStandRange{Scalar(29, 0, 0), Scalar(31, 255, 255)};
    HSVRange bluerawStandRange{Scalar(115, 0, 0), Scalar(125, 255, 255)};

    auto standLocation = findRawStand(im, yellowrawStandRange);
    auto standNumber = YELLOW_STAND;
    if (standLocation.isNull()) {
        standLocation = findRawStand(im, bluerawStandRange);
        standNumber = BLUE_STAND;

        if (standLocation.isNull()) {
            standNumber = NO_STAND;
            ROS_ERROR("No stand found!");
            return StandAngle();
        }
    }

    auto left = ceil(0.3 * standLocation.upperPoint.x + 0.7 * standLocation.lowerPoint.x);
    auto center = floor(0.5 * standLocation.upperPoint.x + 0.5 * standLocation.lowerPoint.x);
    auto right = floor(0.7 * standLocation.upperPoint.x + 0.3 * standLocation.lowerPoint.x);

    auto leftIndex = getRangeIndex(left, scan);
    auto centerIndex = getRangeIndex(center, scan);
    auto rightIndex = getRangeIndex(right, scan);

    auto leftDistance = scan.ranges.at(leftIndex);
    auto centerDistance =scan.ranges.at(centerIndex);
    auto rightDistance = scan.ranges.at(rightIndex);

    // refer to diagram in src folder for annotations
    auto alpha = (rightIndex - leftIndex) * scan.angle_increment;
    auto gamma = leftIndex * scan.angle_increment + scan.angle_min;

    auto x = sqrt(leftDistance * leftDistance + rightDistance * rightDistance -
                  2 * leftDistance * rightDistance * cos(alpha));
    auto beta = acos((leftDistance * leftDistance + rightDistance * rightDistance - x * x) /
                     (2 * leftDistance * rightDistance));

    auto theta = beta - gamma;

    // ROS_INFO("dl: %f metres, dr: %f metres, x: %f metres", leftDistance, rightDistance, x);
    // ROS_INFO("beta: %f radians, gamma: %f radians, theta: %f radians", beta, gamma, theta);
    StandAngle result{standNumber, centerDistance, theta};

    // circle(im, Point(left, 240), 5, Scalar(255, 255, 0), -1);
    // circle(im, Point(center, 240), 5, Scalar(255, 0, 255), -1);
    // circle(im, Point(right, 240), 5, Scalar(0, 255, 255), -1);
    // ROS_INFO("Stand: %f metres @ %f radians", result.standDistance, result.angle);
    return result;
}

Mat StandAngleCalculator::extractSegments(Mat im, HSVRange range) {
    Mat im_HSV;
    cvtColor(im, im_HSV, COLOR_BGR2HSV);

    // Segment
    Mat mask;
    inRange(im_HSV, range.lowerRange, range.upperRange, mask);  // using hsv ranges

    // Clean
    Mat kernel = Mat(10, 10, CV_8UC1, 1);
    morphologyEx(mask, mask, MORPH_CLOSE, kernel);

    return mask;
}

struct str {
    bool operator()(Point2f a, Point2f b) { return a.x <= b.x; }
} comp;

PointInterval StandAngleCalculator::findRawStand(Mat im, HSVRange range) {
    Mat mask = extractSegments(im, range);

    // Find Contours
    std::vector<std::vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // CHoose the largest one
    std::vector<Point> rawStand;
    if (contours.size() == 1) {
        rawStand = contours[0];
    } else if (contours.size() > 1) {
        ROS_ERROR("Oh shit! Found %d contours.", (int)contours.size());
        ROS_INFO("Selecting largest contour.");

        int maxArea = 0;
        int index = 0;

        for (auto i = 0; i < contours.size(); i++) {
            auto area = contourArea(contours[i]);
            if (area > maxArea) {
                maxArea = area;
                index = i;
            }
        }

        rawStand = contours[index];
    } else {
        ROS_ERROR("Oh shit! Found no contours of this color range. Range lower: %f", range.lowerRange[0]);
        return PointInterval();
    }

    auto minmaxResult = std::minmax_element(rawStand.begin(), rawStand.end(), comp);
    return PointInterval(*minmaxResult.first, *minmaxResult.second);

    // Find rectangle shape
    // auto perimeter = arcLength(rawStand, true);
    // std::vector<Point> stand;
    // approxPolyDP(rawStand, stand, perimeter * 0.02, true);

    // Try to find the 4 corners
    // std::sort(stand.begin(), stand.end(), comp);

    // auto lastIndex = stand.size();
    // auto topLeft = stand[0].y < stand[1].y ? stand[0] : stand[1];
    // auto bottomLeft = stand[0].y >= stand[1].y ? stand[0] : stand[1];
    // auto topRight = stand[lastIndex - 2].y < stand[lastIndex - 1].y ? stand[lastIndex - 2] : stand[lastIndex - 1];
    // auto bottomRight = stand[lastIndex - 2].y >= stand[lastIndex - 1].y ? stand[lastIndex - 2] : stand[lastIndex -
    // 1];

    // return PointInterval(stand[0], stand[lastIndex - 1]);
}