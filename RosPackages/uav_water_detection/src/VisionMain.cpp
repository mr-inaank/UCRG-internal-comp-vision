// starter of code to image detect mandarins
#include <iostream>
#include <opencv2/opencv.hpp>

#include <PoolDetector.h>
#include <CheckerboardDetector.h>

using namespace cv;

#define MAX_HEIGHT 800

#define IMAGE_EXT ".png.jpg.jpeg"
#define VIDEO_EXT ".mp4.mov.avi"

int main(int argc, char** argv) {

    if (argc < 2) {
        fprintf(stderr, "please provide an image name, and result name, EXITING\n");
        exit(1);
    }

    char* file_name = argv[1];
    char* file_ext = strrchr(file_name, '.');
    if (file_ext == NULL) {
        fprintf(stderr, "Not a video or image file, EXITING\n");
        exit(1);
    }

    if (strstr(IMAGE_EXT, file_ext) != NULL) {
        printf("Processing an image file\n");
        Mat im = imread(argv[1], IMREAD_COLOR);

        CheckerboardDetector::getCheckerboardLocation(im);
        imshow("asf", im);

        waitKey(0);

    } else if (strstr(VIDEO_EXT, file_ext) != NULL) {
        printf("Processing a video file\n");

        // make a video capture object
        // VideoCapture cap(argv[1]);
        VideoCapture cap(0);

        if (!cap.isOpened()) {
            fprintf(stderr, "Couldn't open the video, EXITING\n");
            exit(1);
        }


        Mat frame;
        cap >> frame;

        while (!frame.empty()) {
            CheckerboardDetector::getCheckerboardLocation(frame);
            imshow("asdf", frame);
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

    return 0;
}