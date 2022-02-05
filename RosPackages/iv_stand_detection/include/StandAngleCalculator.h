#ifndef STANDANGLECALCULATOR_HPP
#define STANDANGLECALCULATOR_HPP

#include <opencv2/opencv.hpp>
#include <sensor_msgs/LaserScan.h>

enum StandType { NO_STAND = 0, BLUE_STAND = 1, YELLOW_STAND = 2 };

struct StandAngle {
    StandAngle(StandType _standType = StandType::NO_STAND, double _standDistance = nan(""), double _angle = nan(""))
        : stand{_standType}, standDistance{_standDistance}, angle{_angle} {};

    StandType stand;
    double standDistance;
    double angle;
};

struct HSVRange {
    HSVRange(cv::Scalar _lowerRange, cv::Scalar _upperRange) : lowerRange{_lowerRange}, upperRange{_upperRange} {};

    cv::Scalar lowerRange;
    cv::Scalar upperRange;
};

struct PointInterval {
    PointInterval(){};

    PointInterval(cv::Point2f pointA, cv::Point2f pointB) : lowerPoint{pointA}, upperPoint{pointB}, isEmpty{false} {
        if (lowerPoint.x > upperPoint.x) {
            auto temp = lowerPoint;
            lowerPoint = upperPoint;
            upperPoint = temp;
        }
    };

    bool isNull() { return isEmpty; };

    cv::Point2f lowerPoint;
    cv::Point2f upperPoint;

   private:
    bool isEmpty = true;
};

class StandAngleCalculator {
   public:
    static StandAngle calcStandAngle(cv::Mat im, const sensor_msgs::LaserScan& scan);

   private:
    static cv::Mat extractSegments(cv::Mat im, HSVRange range);
    static PointInterval findRawStand(cv::Mat im, HSVRange range);
};

#endif