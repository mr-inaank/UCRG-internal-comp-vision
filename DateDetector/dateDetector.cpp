// starter of code to image detect mandarins
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;

#define MAX_HEIGHT 800

#define IMAGE_EXT ".png.jpg.jpeg"
#define VIDEO_EXT ".mp4.mov.avi.webm"

const Point NULL_POINT{-1, -1};

struct HSVValues {
    HSVValues() {};

    int lowH = 0;
    int highH = 255;
    int lowS = 0;
    int highS = 255;
    int lowV = 0;
    int highV = 255;
};

std::ostream& operator<<(std::ostream& os, const HSVValues hsvValues) {
    os << "H: " << hsvValues.lowH << ", " << hsvValues.highH << std::endl;
    os << "S: " << hsvValues.lowS << ", " << hsvValues.highS << std::endl;
    os << "V: " << hsvValues.lowV << ", " << hsvValues.highV << std::endl;

    return os;
}

Point processFrame(Mat im);
static void onMouse(int event, int x, int y, int flags, void* param);

int a;
int b;

int main(int argc, char** argv) {

    // Get the name of the image to test
    if (argc < 2) {
        fprintf(stderr, "please provide an image name EXITING\n");
        exit(1);
    }

    // Check whether it is a still image or video stream
    char* file_name = argv[1];
    char* file_ext = strrchr(file_name, '.');
    if (file_ext == NULL) {
        fprintf(stderr, "Not a video or image file, EXITING\n");
        exit(1);
    }

    String winName{ "ColorPicker" };
    namedWindow(winName, WINDOW_AUTOSIZE);

    HSVValues hsvValues{};
    hsvValues.lowH = 0;
    hsvValues.highH = 255;
    hsvValues.lowS = 130;
    hsvValues.highS = 255;
    hsvValues.lowV = 0;
    hsvValues.highV = 45;

    createTrackbar("LowH", winName, &hsvValues.lowH, 255);
    createTrackbar("HighH", winName, &hsvValues.highH, 255);

    createTrackbar("LowS", winName, &hsvValues.lowS, 255);
    createTrackbar("HighS", winName, &hsvValues.highS, 255);

    createTrackbar("LowV", winName, &hsvValues.lowV, 255);
    createTrackbar("HighV", winName, &hsvValues.highV, 255);

    Mat frame;
    VideoCapture cap;
    bool isVid{ strstr(VIDEO_EXT, file_ext) != NULL };

    if (strstr(IMAGE_EXT, file_ext) != NULL) {
        printf("Processing an image file\n");
        frame = imread(argv[1], 1);

    } else if (strstr(VIDEO_EXT, file_ext) != NULL) {
        printf("Processing a video file\n");

        // make a video capture object
        // cap = VideoCapture(argv[1]);
        cap = VideoCapture(0);

        if (!cap.isOpened()) {
            fprintf(stderr, "Couldn't open the video, EXITING\n");
            exit(1);
        }

        // Read in first frame
        cap >> frame;

    } else {
        fprintf(stderr, "%s is not a video or image file, EXITING\n", file_ext);
        exit(1);
    }

    setMouseCallback(winName, onMouse, &frame);
    std::cout << frame.at<Vec3b>(255, 255) << std::endl;
    while (!frame.empty()) {
        // process and show the frame
        Point result = processFrame(frame);
        std::cout << result << std::endl;
        circle(frame, result * 2, 30, Scalar(255, 0, 0), -1);
        imshow("winName", frame);

        char c = static_cast<char>(waitKey(25));
        if (c == 27) {
            break;
        } else if (c == 'a') std::cout << hsvValues << std::endl;

        if (isVid) {
            cap >> frame;
        }
    }

    if (isVid) {
        cap.release();
    }

    std::cout << hsvValues << std::endl;
}


Mat getDateMask(Mat im) {
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

Point processFrame(Mat im) {

    while (im.rows > MAX_HEIGHT) {
        resize(im, im, Size(), 0.5, 0.5, INTER_LINEAR);
    }
    std::cout << b << ", " << a << std::endl;

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
    // circle(im, center, 5, Scalar(255, 0, 0), 1);

    return center;

    

   /* // Setup SimpleBlobDetector parameters.
    SimpleBlobDetector::Params params;

    // Change thresholds
    params.minThreshold = 10;
    params.maxThreshold = 200;

    // // Filter by Area.
    params.filterByArea = false;
    params.minArea = 300;

    // // Filter by Circularity
    params.filterByCircularity = false;
    params.minCircularity = 0.1;

    // // Filter by Convexity
    params.filterByConvexity = false;
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

    Mat im_with_keypoints;
    drawKeypoints(im, keypoints, im_with_keypoints, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);*/

    Mat result{ mask };
    bitwise_and(im, im, result, mask = mask);

    // return im;
    // return mask;
    // return result;
}

static void onMouse(int event, int x, int y, int flags, void* param) {
    Mat raw = *((Mat*)param); //cast and deref the param

    if (event == EVENT_LBUTTONDOWN) {
        a = x;
        b = y;

        // cvtColor(raw, img, COLOR_BGR2HSV);
        // std::cout << img.rows << ",  " << img.cols << std::endl;
        // Vec3b val = img.at<Vec3b>(y, x); // opencv is row-major ! 
        // std::cout << "x= " << x << " y= " << y << " val= " << val << std::endl;
    }
}
