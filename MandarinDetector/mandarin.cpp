#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;

#define MAX_HEIGHT 800

#define IMAGE_EXT ".png.jpg.jpeg"
#define VIDEO_EXT ".mp4.mov.avi"

void processFrame(Mat im);
Mat getMandarinMask(Mat im);
std::vector<Point> findMandarins(Mat im);

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
        waitKey(0);

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

    auto result = findMandarins(im);

    for (auto pt : result) {
        circle(im, pt, 3, Scalar(0, 255, 0), -1);
    }
    imshow("iim", im);
}



// ========================================================================
// =================== YOU NEED THE BELOW FUNCTIONS =======================
// ========================================================================
Mat getMandarinMask(Mat im) {
    Mat im_HSV;
    cvtColor(im, im_HSV, COLOR_BGR2HSV);

    // Segment
    Mat mask;
    auto lowerRange = Scalar(6, 115, 0);
    auto upperRange = Scalar(11, 255, 255);
    inRange(im_HSV, lowerRange, upperRange, mask); // using hsv ranges

    // Clean
    Mat kernel = Mat(10, 10, CV_8UC1, 1);
    morphologyEx(mask, mask, MORPH_CLOSE, kernel);

    // imshow("mask", mask);
    return mask;
}

std::vector<Point> findMandarins(Mat im) {
    Mat mask = getMandarinMask(im);

    // Setup SimpleBlobDetector parameters.
    SimpleBlobDetector::Params params;

    // Change thresholds
    params.minThreshold = 10;
    params.maxThreshold = 200;

    // // Filter by Area.
    params.filterByArea = true;
    params.minArea = 50;
    params.maxArea = im.rows * im.cols / 5;

    // // Filter by Circularity
    params.filterByCircularity = true;
    params.minCircularity = 0.1;

    // // Filter by Convexity
    params.filterByConvexity = true;
    params.minConvexity = 0.75;

    // // Filter by Inertia
    params.filterByInertia = true;
    params.minInertiaRatio = 0.01;


    // Storage for blobs
    std::vector<KeyPoint> keypoints;

    // Set up detector with params
    Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);

    // Detect blobs

    // change the image to black on white background 
    // (just need to get rid of white points)
    bitwise_not(mask, mask);

    detector->detect(mask, keypoints);

    // drawKeypoints(im, keypoints, im, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    std::cout << keypoints.size() << std::endl;

    std::vector<Point> result;
    for (auto kp : keypoints) {
        result.push_back(kp.pt);
    }

    auto center = Point(im.rows - 1, im.cols - 1);
    std::sort(result.begin(), result.end(),
        [=](Point a, Point b) {
            auto diffA = center - a;
            auto distA = (diffA).x * (diffA).x + (diffA).y + (diffA).y;

            auto diffB = center - b;
            auto distB = (diffB).x * (diffB).x + (diffB).y + (diffB).y;

            return (distA < distB);
        }
    );

    return result;
}