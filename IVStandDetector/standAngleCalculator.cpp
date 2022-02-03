#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;

#define MAX_HEIGHT 800

#define IMAGE_EXT ".png.jpg.jpeg"
#define VIDEO_EXT ".mp4.mov.avi"

enum StandType {
    NO_STAND = 0,
    YELLOW_STAND = 1,
    BLUE_STAND = 2
};

struct HSVRange {
    HSVRange(Scalar _lowerRange, Scalar _upperRange)
        : lowerRange{ _lowerRange }, upperRange{ _upperRange } {};

    Scalar lowerRange;
    Scalar upperRange;
};

struct PointInterval {
    PointInterval() {};

    PointInterval(Point2f pointA, Point2f pointB)
        : lowerPoint{ pointA }, upperPoint{ pointB }, isEmpty{ false } {

        if (lowerPoint.x > upperPoint.x) {
            auto temp = lowerPoint;
            lowerPoint = upperPoint;
            upperPoint = temp;
        }
    };

    bool isNull() {
        return isEmpty;
    };

    Point2f lowerPoint;
    Point2f upperPoint;

private:
    bool isEmpty = true;
};

void processFrame(Mat im);
Mat extractSegments(Mat im, HSVRange range);
PointInterval findRawStand(Mat im, HSVRange range);

int main(int argc, char** argv) {

    // Get the name of the image to test
    if (argc < 2) {
        fprintf(stderr, "please provide an image name, and result name, EXITING\n");
        exit(1);
    }

    // Check whether it is a still image or video stream
    char* file_name = argv[1];
    char* file_ext = strrchr(file_name, '.');
    if (file_ext == NULL) {
        fprintf(stderr, "Not a video or image file, EXITING\n");
        exit(1);
    }

    if (strstr(IMAGE_EXT, file_ext) != NULL) {
        printf("Processing an image file\n");
        // TODO: check that argv[2] is a valid name
        // is an image file
        // Read image
        Mat im = imread(argv[1], IMREAD_COLOR);

        // cout << im.rows << " rows\n";
        // cout << im.cols << " columns\n";

        processFrame(im);

    } else if (strstr(VIDEO_EXT, file_ext) != NULL) {
        printf("Processing a video file\n");

        // make a video capture object
        VideoCapture cap(argv[1]);

        if (!cap.isOpened()) {
            fprintf(stderr, "Couldn't open the video, EXITING\n");
            exit(1);
        }

        // read in video
        Mat frame;
        // Capture frame-by-frame
        cap >> frame;

        while (!frame.empty()) {
            processFrame(frame);

            char c = (char)waitKey(25);
            if (c == 27) {
                break;
            }

            cap >> frame;
        }
        cap.release();
    } else {
        fprintf(stderr, "%s is not a video or image file, EXITING\n", file_ext);
        exit(1);
    }
}

void processFrame(Mat im) {

    while (im.rows > MAX_HEIGHT) {
        resize(im, im, Size(), 0.5, 0.5, INTER_LINEAR);
    }

    HSVRange yellowrawStandRange{ Scalar(29,0,0), Scalar(31,255,255) };
    HSVRange bluerawStandRange{ Scalar(115,0,0), Scalar(125,255,255) };


    auto result = findRawStand(im, yellowrawStandRange);
    auto standNumber = YELLOW_STAND;
    if (result.isNull()) {
        result = findRawStand(im, bluerawStandRange);
        standNumber = BLUE_STAND;

        if (result.isNull()) {
            standNumber = NO_STAND;
            std::cerr << "No stand found!" << std::endl;
            throw std::invalid_argument("No stand found.");
        }
    }

    std::cout << (result.upperPoint - result.lowerPoint).x << std::endl;
}


Mat extractSegments(Mat im, HSVRange range) {
    Mat im_HSV;
    cvtColor(im, im_HSV, COLOR_BGR2HSV);

    // Segment
    Mat mask;
    inRange(im_HSV, range.lowerRange, range.upperRange, mask); // using hsv ranges

    // Clean
    Mat kernel = Mat(10, 10, CV_8UC1, 1);
    morphologyEx(mask, mask, MORPH_CLOSE, kernel);

    return mask;
}

struct str {
    bool operator() (Point2f a, Point2f b) {
        return a.x <= b.x;
    }
} comp;


PointInterval findRawStand(Mat im, HSVRange range) {
    Mat mask = extractSegments(im, range);

    // Find Contours
    std::vector<std::vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // CHoose the largest one
    std::vector<Point> rawStand;
    if (contours.size() == 1) {
        rawStand = contours[0];
    } else if (contours.size() > 1) {
        std::cerr << "Oh shit! Found " << contours.size() << " contours." << std::endl;
        std::cout << "Selecting largest contour." << std::endl;

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
        std::cerr << "Oh shit! Found no contours of this color range. Range lower: " << range.lowerRange << std::endl;
        return PointInterval();
    }

    // Find rectangle shape
    auto perimeter = arcLength(rawStand, true);
    std::vector<Point> stand;
    approxPolyDP(rawStand, stand, perimeter * 0.02, true);

    // Try to find the 4 corners
    std::sort(stand.begin(), stand.end(), comp);

    auto lastIndex = stand.size();
    auto topLeft = stand[0].y < stand[1].y ? stand[0] : stand[1];
    auto bottomLeft = stand[0].y >= stand[1].y ? stand[0] : stand[1];
    auto topRight = stand[lastIndex - 2].y < stand[lastIndex - 1].y ? stand[lastIndex - 2] : stand[lastIndex - 1];
    auto bottomRight = stand[lastIndex - 2].y >= stand[lastIndex - 1].y ? stand[lastIndex - 2] : stand[lastIndex - 1];

    // Calculate depth????  
    return PointInterval(
        stand[0], stand[lastIndex - 1]
    );
}