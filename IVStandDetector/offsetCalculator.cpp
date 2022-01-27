#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;

#define MAX_HEIGHT 800

#define IMAGE_EXT ".png.jpg.jpeg"
#define VIDEO_EXT ".mp4.mov.avi"

Point2f processFrame(Mat im);
Mat extractSegments(Mat im);
Point2f findRedDot(Mat im);

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

Point2f processFrame(Mat im) {

    while (im.rows > MAX_HEIGHT) {
        resize(im, im, Size(), 0.5, 0.5, INTER_LINEAR);
    }

    Point2f result = findRedDot(im);

    auto centerLine = (static_cast<double>(im.cols)) / 2.0;

    std::cout << result.x - centerLine << std::endl;

    return result;
}


Mat extractSegments(Mat im) {
    Mat im_HSV;
    cvtColor(im, im_HSV, COLOR_BGR2HSV);

    // Segment
    Scalar lower_range, upper_range;
    lower_range = Scalar(0, 180, 0);
    upper_range = Scalar(5, 255, 255);
    Mat mask;
    inRange(im_HSV, lower_range, upper_range, mask); // using hsv ranges

    // Clean
    Mat kernel = Mat(10, 10, CV_8UC1, 1);
    morphologyEx(mask, mask, MORPH_CLOSE, kernel);

    return mask;
}

Point2f findRedDot(Mat im) {
    Mat mask = extractSegments(im);

    SimpleBlobDetector::Params params;

    params.filterByArea = true;
    params.minArea = 10;

    params.filterByCircularity = true;
    params.minCircularity = 0.9;

    params.filterByConvexity = true;
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

    if (keypoints.size() != 1) {
        std::cerr << "Oh shit! Found " << keypoints.size() << " keypoints." << std::endl;
        throw std::invalid_argument("Invaild number of keypoints found");
    }

    return keypoints[0].pt;
}