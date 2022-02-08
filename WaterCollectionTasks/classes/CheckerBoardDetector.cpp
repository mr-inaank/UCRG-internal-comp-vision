#include <CheckerBoardDetector.h>
using namespace cv;

Mat CheckerBoardDetector::preprocessImage(Mat im) {
    Mat gray;
    cvtColor(im, gray, COLOR_BGR2GRAY);

    // Mat clearingMask = Mat(im.rows, im.cols, CV_8UC1, Scalar(0));
    // clearingMask(Rect(Point(0, (int)im.rows * 0.7), Point(im.cols, im.rows))) = 255;
    // Mat mask;
    // bitwise_and(mask1, clearingMask, mask);

    Mat mask;
    threshold(gray, mask, 255, 255, THRESH_OTSU);

    Mat result1;
    cornerHarris(mask, result1, 3, 5, 0.2);
    result1.convertTo(result1, CV_8UC1, 255, 0);

    threshold(result1, result1, 255, 255, THRESH_OTSU);

    Mat kernel = getStructuringElement(MORPH_RECT, Size(9, 27));
    morphologyEx(result1, result1, MORPH_CLOSE, kernel);
    kernel = getStructuringElement(MORPH_RECT, Size(35, 9));
    morphologyEx(result1, result1, MORPH_CLOSE, kernel);
    kernel = getStructuringElement(MORPH_RECT, Size(11, 11));
    morphologyEx(result1, result1, MORPH_OPEN, kernel);
    kernel = getStructuringElement(MORPH_RECT, Size(19, 19));
    morphologyEx(result1, result1, MORPH_CLOSE, kernel);

    return result1;
}

Point CheckerBoardDetector::getCheckerboardCenter(Mat im) {
    Mat result1 = preprocessImage(im);
    std::vector<std::vector<Point>> contours;
    findContours(result1, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    int index = -1;
    double maxArea = 0;

    for (auto i = 0; i < contours.size(); i++) {
        auto area = contourArea(contours[i]);
        if (area >= 4000) {
            if (maxArea < area) {
                index = i;
                maxArea = area;
            }
        }
    }

    if (index < 0) {
        return NULL_POINT;
    }

    Rect rect = boundingRect(contours[index]);
    // rectangle(im, rect, Scalar(255, 0, 0), 2);
    Point center = (rect.br() + rect.tl()) * 0.5;
    // circle(im, center, 5, Scalar(255, 0, 0), 1);

    return center;
}





#define MAX_HEIGHT 800

#define IMAGE_EXT ".png.jpg.jpeg"
#define VIDEO_EXT ".mp4.mov.avi"

int main(int argc, char** argv) {

    // Get the name of the image to test
    if (argc < 2) {
        fprintf(stderr, "please provide an image name, EXITING\n");
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

        // Read image
        Mat im = imread(argv[1], IMREAD_COLOR);

        Point result = CheckerBoardDetector::getCheckerboardCenter(im);
        if (result != NULL_POINT) {
            circle(im, result, 5, Scalar(0, 255, 0), 2);
        }

        imshow("result", im);

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

        // set up the writer
        printf("about to rescale, %d, %d\n", frame.rows, frame.cols);
        int div_factor = 1;
        int rows = frame.rows;
        while (rows > MAX_HEIGHT) {
            rows = rows / 2;
            div_factor = div_factor * 2;
        }
        int cols = frame.cols / div_factor;

        while (!frame.empty()) {

            while (frame.rows > MAX_HEIGHT) {
                resize(frame, frame, Size(), 0.5, 0.5, INTER_LINEAR);
            }

            // process and show the frame
            Point result = CheckerBoardDetector::getCheckerboardCenter(frame);
            if (result != NULL_POINT) {
                circle(frame, result, 5, Scalar(0, 255, 0), 2);
            }

            imshow("result", frame);
            char c = (char)waitKey(0.1 * 1000);
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