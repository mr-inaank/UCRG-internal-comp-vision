// starter of code to image detect mandarins
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;

#define MAX_HEIGHT 800

#define IMAGE_EXT ".png.jpg.jpeg"
#define VIDEO_EXT ".mp4.mov.avi.webm"

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


Mat processFrame(Mat im, HSVValues hsvValues);
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
    hsvValues.lowS = 0;
    hsvValues.highS = 255;
    hsvValues.lowV = 150;
    hsvValues.highV = 255;

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
        Mat result = processFrame(frame, hsvValues);
        imshow(winName, result);
        imshow("winName", frame);

        char c = static_cast<char>(waitKey(1));
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


Mat processFrame(Mat im, HSVValues hsvValues) {

    while (im.rows > MAX_HEIGHT) {
        resize(im, im, Size(), 0.5, 0.5, INTER_LINEAR);
    }

    // Convert the images to HSV channels to filter by colour
    Mat im_HSV;
    cvtColor(im, im_HSV, COLOR_BGR2HSV);

    // add a colour masks to only icnclude certain hue range
    Scalar lower_bound, upper_bound;
    lower_bound = Scalar(hsvValues.lowH, hsvValues.lowS, hsvValues.lowV);
    upper_bound = Scalar(hsvValues.highH, hsvValues.highS, hsvValues.highV);
    Mat mask;
    inRange(im_HSV, lower_bound, upper_bound, mask); // using hsv ranges

    Mat kernel = getStructuringElement(MORPH_RECT, Size(15, 15));
    morphologyEx(mask, mask, MORPH_ERODE, kernel);

    std::vector<std::vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    Mat result = Mat(im.rows, im.cols, CV_8UC1, Scalar(0));
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

    kernel = getStructuringElement(MORPH_RECT, Size(31, 31));
    morphologyEx(result, result, MORPH_CLOSE, kernel);
    morphologyEx(result, result, MORPH_OPEN, kernel);

    std::vector<std::vector<Point>> contours2;
    findContours(result, contours2, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    auto index = -1;
    auto maxArea = 0;
    for (auto i = 0; i < contours2.size(); i++) {
        auto area = contourArea(contours2[i]);
        if (area < 3000 || area < maxArea) {
            continue;
        }

        maxArea = area;
        index = i;
    }

    if (index >= 0) {
        auto rect = boundingRect(contours2[index]);
        rectangle(im, rect, Scalar(0, 0, 255), 3);
    }


    imshow("imas", im);
    return mask;
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
