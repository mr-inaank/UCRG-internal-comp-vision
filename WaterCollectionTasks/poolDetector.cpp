// starter of code to image detect mandarins
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;

#define MAX_HEIGHT 800

#define IMAGE_EXT ".png.jpg.jpeg"
#define VIDEO_EXT ".mp4.mov.avi"
const auto NULL_POINT = Point(-1, -1);

Point processFrame(Mat im);
void printInstruction(Mat im, Point poolCenter);

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

            imshow("result", frame);
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

    // if it were capturing from a camera:

}

Mat getPoolMask(Mat im) {
    // Convert the images to HSV channels to filter by colour
    Mat im_HSV;
    cvtColor(im, im_HSV, COLOR_BGR2HSV);

    // add a colour masks to only include certain hue range
    Scalar lower_mask, upper_mask;
    lower_mask = Scalar(100, 85, 75);
    upper_mask = Scalar(120, 255, 255);

    Mat mask;
    inRange(im_HSV, lower_mask, upper_mask, mask); // using hsv ranges

    Mat kernel = getStructuringElement(MORPH_RECT, Size(7, 7));
    morphologyEx(mask, mask, MORPH_OPEN, kernel);
    morphologyEx(mask, mask, MORPH_OPEN, kernel);
    morphologyEx(mask, mask, MORPH_CLOSE, kernel);

    return mask;
}

std::vector<Point> extractPoolContour(Mat mask) {
    std::vector<std::vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    std::vector<Point> pool;

    if (contours.size() <= 0) {
        return pool;
    }

    // If there are too many contours, we select largest one.
    auto maxArea = 0;
    auto index = 0;
    for (auto i = 0; i < contours.size(); i++) {
        auto area = contourArea(contours[i]);
        if (area > maxArea) {
            maxArea = area;
            index = i;
        }
    }

    if (maxArea > 3000) {
        pool = contours[index];
    }

    return pool;
}

Point getPoolCenter(std::vector<Point> pool) {
    auto M = moments(pool);
    auto cX = (int)(M.m10 / M.m00);
    auto cY = (int)(M.m01 / M.m00);

    Point result{ cX,cY };
    std::cout << result << std::endl;
    return result;
}

void printInstruction(Mat im, Point poolCenter) {
    const auto xOffset = 0;
    const auto yOffset = 0;

    std::string horizontal;
    std::string vertical;

    if (poolCenter.x < im.cols / 2 + xOffset) {
        horizontal = "LEFT ";
    } else if (poolCenter.x > im.cols / 2 + xOffset) {
        horizontal = "RIGHT ";
    }

    if (poolCenter.y < im.rows / 2 + yOffset) {
        vertical = "FORWARD ";
    } else if (poolCenter.y > im.rows / 2 + yOffset) {
        vertical = "BACKWARD ";
    }

    std::cout
        << "Command: "
        << horizontal << abs(im.cols / 2 + xOffset - poolCenter.x)
        << ", "
        << vertical << abs(im.rows / 2 + yOffset - poolCenter.y)
        << std::endl;
}

Point processFrame(Mat im) {
    Mat mask = getPoolMask(im);

    // Mat result;
    // bitwise_and(im, im, result, mask = mask);

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
}
