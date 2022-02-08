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
    hsvValues.highS = 33;
    hsvValues.lowV = 123;
    hsvValues.highV = 188;

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
        cap = VideoCapture(argv[1]);

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
    // std::cout << im_HSV.at<Vec3b>(b,a) << std::endl;
    std::cout << b << ", " << a << std::endl;

    // TODO: clean up edges inside of the white areas
    // open to get rid of white, masked out points
    Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    morphologyEx(mask, mask, MORPH_OPEN, kernel);
    // morphologyEx(mask, mask, MORPH_OPEN, kernel);
    // morphologyEx(mask, mask, MORPH_CLOSE, kernel);



    Mat result{ mask };
    bitwise_and(im, im, result, mask = mask);

    return mask;
    return result;
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
