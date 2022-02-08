// starter of code to image detect mandarins
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;

#define MAX_HEIGHT 800

#define IMAGE_EXT ".png.jpg.jpeg"
#define VIDEO_EXT ".mp4.mov.avi"
const auto NULL_POINT = Point(-1, -1);

auto prevCenter = NULL_POINT;

Point processFrame(Mat im);
void printInstruction(Mat im, Point poolCenter){};

RNG rng(12345);

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

        Point result = processFrame(im);
        if (result != NULL_POINT) {
            printInstruction(im, result);

            circle(im, result, 5, Scalar(0, 255, 0), 2);

        }

        // imshow("result", im);

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
            Point result = processFrame(frame);
            if (result != NULL_POINT) {
                printInstruction(frame, result);

                circle(frame, result, 5, Scalar(0, 255, 0), 2);

            }

            // imshow("result", frame);
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

Mat preprocessImage(Mat im) {
    // Convert the images to HSV channels to filter by colour
    Mat im_HSV, gray;
    cvtColor(im, im_HSV, COLOR_BGR2HSV);
    cvtColor(im, gray, COLOR_BGR2GRAY);

    // add a colour masks to only include certain hue range
    Scalar lower_mask, upper_mask;
    Scalar lower_mask2, upper_mask2;
    lower_mask = Scalar(0, 0, 0);
    upper_mask = Scalar(255, 11, 255);
    lower_mask2 = Scalar(0, 4, 144);
    upper_mask2 = Scalar(255, 45, 255);

    Mat mask1, mask2;
    inRange(im_HSV, lower_mask, upper_mask, mask1); // using hsv ranges
    inRange(im_HSV, lower_mask2, upper_mask2, mask2); // using hsv ranges

    Mat mask = mask1.clone();

    Mat kernel = getStructuringElement(MORPH_RECT, Size(11, 11));
    morphologyEx(mask, mask, MORPH_OPEN, kernel);

    Mat result1;
    cornerHarris(mask, result1, 3, 5, 0.1);
    result1.convertTo(result1, CV_8UC1, 255, 0);

    imshow("mask2", result1);
    kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(result1, result1, MORPH_CLOSE, kernel);
    kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    morphologyEx(result1, result1, MORPH_ERODE, kernel);

    std::vector<Point> points;
    findNonZero(result1, points);

    Rect rect = boundingRect(points);
    if (rect.area() > 10000) {
        rectangle(im, rect, Scalar(0, 0, 255), 2);

        Point center = (rect.br() + rect.tl()) * 0.5;
        circle(im, center, 5, Scalar(0, 255, 0), 1);
        auto lr = 0.3;
        if (prevCenter != NULL_POINT) {
            center = center * lr + prevCenter * (1 - lr);
        }
        circle(im, center, 7, Scalar(0, 255, 0), 3);
        prevCenter = center;
    } else {
        prevCenter = NULL_POINT;
    }

    imshow("mask1", result1);
    return mask;
}

Point processFrame(Mat im) {
    Mat mask = preprocessImage(im);
    /*
        // Mat result;
        // bitwise_and(im, im, result, mask = mask);
        circle(im, Point(10, 10), 5, Scalar(0, 255, 0), 2);
        auto pool = extractPoolContour(mask);
        if (pool.size() <= 0) {
            return NULL_POINT;
        }

        drawContours(im, std::vector<std::vector<Point>>{pool}, -1, Scalar(30, 70, 250), 2);

        // auto perimeter = arcLength(contours[index], true);
        // auto area = contourArea(pool);
        // auto circularity = 4 * CV_PI * area / (perimeter * perimeter);
        // std::cout << circularity << ", " << area << std::endl;

        auto poolCenter = getPoolCenter(pool);
        circle(im, poolCenter, 5, Scalar(0, 255, 0), 2);
        return poolCenter;
    */
    // im = mask;
    imshow("result", mask);
    imshow("result2", im);
    return NULL_POINT;
}
