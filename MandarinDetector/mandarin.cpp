// starter of code to image detect mandarins
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;

#define MAX_HEIGHT 800

#define IMAGE_EXT ".png.jpg.jpeg"
#define VIDEO_EXT ".mp4.mov.avi"

Mat processFrame(Mat im, int show_proc);

int main(int argc, char** argv) {

    // Get the name of the image to test
    if (argc < 3) {
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

        processFrame(im, true);

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
        // TODO: check that argv[2] is a valid name
        printf("about to rescale, %d, %d\n", frame.rows, frame.cols);
        int div_factor = 1;
        int rows = frame.rows;
        while (rows > MAX_HEIGHT) {
            rows = rows / 2;
            div_factor = div_factor * 2;
        }
        int cols = frame.cols / div_factor;
        VideoWriter video(argv[2], VideoWriter::fourcc('m', 'p', '4', 'v'), 40, Size(cols, rows));
        // printf("set up the writer!\n");

        while (!frame.empty()) {

            // process and show the frame
            Mat result = processFrame(frame, true);
            video.write(result);

            char c = (char)waitKey(25);
            if (c == 27) {
                break;
            }

            cap >> frame;
        }

        video.release();
        cap.release();
    } else {
        fprintf(stderr, "%s is not a video or image file, EXITING\n", file_ext);
        exit(1);
    }

    // if it were capturing from a camera:

}


Mat processFrame(Mat im, int show_proc) {

    while (im.rows > MAX_HEIGHT) {
        resize(im, im, Size(), 0.5, 0.5, INTER_LINEAR);
    }

    // Convert the images to HSV channels to filter by colour
    Mat im_HSV;
    cvtColor(im, im_HSV, COLOR_BGR2HSV);

    // add a colour masks to only include certain hue range
    Scalar lower_mask, upper_mask;
    // lower_mask = Scalar(5, 70, 70);
    // upper_mask = Scalar(45, 255, 255);
    lower_mask = Scalar(70, 115, 0);
    upper_mask = Scalar(75, 255, 255);
    Mat mask;
    // inRange(im, Scalar(20, 40, 50), Scalar(60, 220, 255), mask); // using hsv ranges
    inRange(im_HSV, lower_mask, upper_mask, mask); // using hsv ranges

    // TODO: clean up edges inside of the white areas
    // open to get rid of white, masked out points
    Mat kernel = Mat(10, 10, CV_8UC1, 1);//getStructuringElement(MORPH_RECT, (5, 5));
    // cout << "made kernel\n";
    morphologyEx(mask, mask, MORPH_CLOSE, kernel);
    // cout << "done morphology\n";	

    Mat result;
    bitwise_and(im, im, result, mask = mask);

    if (show_proc) {
        namedWindow("mask result", WINDOW_AUTOSIZE);
        imshow("mask result", mask);
        namedWindow("result", WINDOW_AUTOSIZE);
        imshow("result", result);
    }


    // Setup SimpleBlobDetector parameters.
    SimpleBlobDetector::Params params;

    // Change thresholds
    params.minThreshold = 10;
    params.maxThreshold = 200;

    // // Filter by Area.
    params.filterByArea = true;
    params.minArea = im.rows * im.cols / 2500;
    params.maxArea = im.rows * im.cols / 25;

    std::cout << params.minArea << ", " << params.maxArea << std::endl;

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

    Mat im_with_keypoints;
    drawKeypoints(im, keypoints, im_with_keypoints, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    std::cout << keypoints.size() << std::endl;
    // Show blobs and just wait
    imshow("keypoints", im_with_keypoints);

    return im_with_keypoints;
}
