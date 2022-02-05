// // starter of code to image detect mandarins
// #include <iostream>
// #include <opencv2/opencv.hpp>

// using namespace cv;

// #define MAX_HEIGHT 800

// #define IMAGE_EXT ".png.jpg.jpeg"
// #define VIDEO_EXT ".mp4.mov.avi"
// const auto NULL_POINT = Point(-1, -1);
// enum Method {
//     RING_DILATE, RING_ERODE, RING_OPEN, RING_CLOSE
// };

// // Point processFrame(Mat im);
// // void printInstruction(Mat im, Point poolCenter);
// std::vector<std::vector<int>> findCandidates(cv::Mat img, int windowWidth, double tolerance);
// int getROI(cv::Mat img, int x, int y, int windowWidth, double alpha, std::vector<std::vector<int>>& storage);
// int getLayer(cv::Mat roi, int layerNum, double  kappa = 0.125); //kappa = 0.125
// std::vector<double> ringMorphologyEx(std::vector<double> img, int size, Method method);


// RNG rng(12345);

// int main(int argc, char** argv) {

//     // Get the name of the image to test
//     if (argc < 2) {
//         fprintf(stderr, "please provide an image name, EXITING\n");
//         exit(1);
//     }


//     // Check whether it is a still image or video stream
//     char* file_name = argv[1];
//     char* file_ext = strrchr(file_name, '.');
//     if (file_ext == NULL) {
//         fprintf(stderr, "Not a video or image file, EXITING\n");
//         exit(1);
//     }

//     if (strstr(IMAGE_EXT, file_ext) != NULL) {
//         printf("Processing an image file\n");
//         // Read image
//         Mat im = imread(argv[1], IMREAD_COLOR);

//         while (im.rows > MAX_HEIGHT) {
//             resize(im, im, Size(), 0.5, 0.5, INTER_LINEAR);
//         }
//         findCandidates(im, 15, 0.6);
//         // if (result != NULL_POINT) {
//         //     printInstruction(im, result);

//         //     circle(im, result, 5, Scalar(0, 255, 0), 2);

//         // }

//         imshow("result", im);

//         waitKey(0);

//     } else if (strstr(VIDEO_EXT, file_ext) != NULL) {
//         printf("Processing a video file\n");

//         // make a video capture object
//         VideoCapture cap(argv[1]);

//         if (!cap.isOpened()) {
//             fprintf(stderr, "Couldn't open the video, EXITING\n");
//             exit(1);
//         }


//         // read in video
//         Mat frame;
//         // Capture frame-by-frame
//         cap >> frame;

//         // set up the writer
//         printf("about to rescale, %d, %d\n", frame.rows, frame.cols);
//         int div_factor = 1;
//         int rows = frame.rows;
//         while (rows > MAX_HEIGHT) {
//             rows = rows / 2;
//             div_factor = div_factor * 2;
//         }
//         int cols = frame.cols / div_factor;

//         while (!frame.empty()) {

//             while (frame.rows > MAX_HEIGHT) {
//                 resize(frame, frame, Size(), 0.5, 0.5, INTER_LINEAR);
//             }

//             // process and show the frame
//             // Point result = processFrame(frame);
//             // if (result != NULL_POINT) {
//                 // printInstruction(frame, result);

//                 // circle(frame, result, 5, Scalar(0, 255, 0), 2);

//             // }
//             findCandidates(frame, 15, 0.6);

//             imshow("result", frame);
//             char c = (char)waitKey(25);
//             if (c == 27) {
//                 break;
//             }

//             cap >> frame;
//         }

//         cap.release();
//     } else {
//         fprintf(stderr, "%s is not a video or image file, EXITING\n", file_ext);
//         exit(1);
//     }

//     // if it were capturing from a camera:

// }


// std::vector<std::vector<int>> findCandidates(cv::Mat img, int windowWidth, double tolerance) {
//     std::vector<std::vector<int>> data(img.rows, std::vector<int>(img.cols, 0));


//       // Convert the images to HSV channels to filter by colour
//     Mat im_HSV, gray;
//     cvtColor(img, im_HSV, COLOR_BGR2HSV);
//     cvtColor(img, gray, COLOR_BGR2GRAY);

//     // add a colour masks to only include certain hue range
//     Scalar lower_mask, upper_mask;
//     lower_mask = Scalar(0, 0, 0);
//     upper_mask = Scalar(255, 11, 255);

//     Mat mask;
//     inRange(im_HSV, lower_mask, upper_mask, mask); // using hsv ranges

//     Mat kernel = getStructuringElement(MORPH_RECT, Size(11, 11));
//     morphologyEx(mask, mask, MORPH_OPEN, kernel);
//     // morphologyEx(mask, mask, MORPH_OPEN, kernel);
//     kernel = getStructuringElement(MORPH_RECT, Size(51, 51));
//     morphologyEx(mask, mask, MORPH_CLOSE, kernel);

//     kernel = getStructuringElement(MORPH_RECT, Size(151, 151));
//     morphologyEx(mask, mask, MORPH_CLOSE, kernel);

//     // Mat result;
//     bitwise_and(img, img , img, mask);

//     for (int i = 0; i < img.rows; i++) {
//         // data[i] = (int*) calloc(img.cols, sizeof *data[i]);
//         for (int j = 0; j < img.cols; j++) {
//             // data[i][j] = 0;
//             getROI(img, i, j, windowWidth, tolerance, data);
//         }
//         std::cout << img.rows - i << std::endl;
//     }

//     // clusterPoints(&data, 3);
//     // removeIsolatedPts(&data, 35, 5);
//     for (int i = 0; i < data.size(); i++) {
//         for (int j = 0; j < data[i].size(); j++) {
//             if (data[i][j] == 1) {
//                 cv::circle(img, cv::Point2d(j, i), 1, cv::Scalar(0, 255, 0));
//             }
//         }
//     }

//     return data;
// }

// int getROI(cv::Mat img, int x, int y, int windowWidth, double alpha, std::vector<std::vector<int>>& storage) {
//     int space = (windowWidth - 1) / 2;

//     int minX = x - space;
//     int minY = y - space;
//     int maxX = x + space;
//     int maxY = y + space;

//     if (space <= 0 || minX < 0 || minY < 0 || maxX >= img.rows || maxY >= img.cols) {
//         return 0;
//     }

//     cv::Mat roi = img(cv::Range(minX, maxX + 1), cv::Range(minY, maxY + 1));

//     int number = 0;
//     for (int i = 1; i < space; i++) {
//         if (getLayer(roi, i)) {
//             number++;
//         }
//     }

//     if (number >= alpha * space) {
//         (storage)[x][y] = 1;
//         return number;
//     } else {
//         return 0;
//     }

// }

// int getLayer(cv::Mat roi, int layerNum, double  kappa) {
//     int size = roi.rows;
//     int center = size / 2;

//     int low = center - layerNum;
//     int high = center + layerNum;

//     std::vector<double> layer;
//     double mean = 0.0;
//     for (int i = low; i < high; i++) {
//         layer.push_back((double)roi.at<uchar>(low, i));
//         mean += layer.back();
//     }
//     for (int i = low; i < high; i++) {
//         layer.push_back((double)roi.at<uchar>(i, high));
//         mean += layer.back();
//     }
//     for (int i = high; i > low; i--) {
//         layer.push_back((double)roi.at<uchar>(high, i));
//         mean += layer.back();
//     }
//     for (int i = high; i > low; i--) {
//         layer.push_back((double)roi.at<uchar>(i, low));
//         mean += layer.back();
//     }
//     mean /= layer.size();
//     // double sum = std::accumulate(std::begin(layer), std::end(layer), 0.0);
//     // double mean =  sum / layer.size();
//     // Taken from stack overflow https://stackoverflow.com/a/12405793
//     double accum = 0.0;
//     std::for_each(std::begin(layer), std::end(layer), [&](const double d) {
//         accum += (d - mean) * (d - mean);
//         });

//     double stddev = sqrt(accum / (layer.size())) / 255.;

//     if (stddev < 0.01) {
//         return 0;
//     }

//     for (int i = 0; i < layer.size(); i++) {
//         layer[i] = layer[i] >= mean ? 1 : 0;
//     }

//     if (layerNum >= 2) {
//         size = kappa * layer.size();
//         if (size % 2 == 0) {
//             size++;
//         }

//         layer = ringMorphologyEx(layer, size, Method::RING_OPEN);
//         layer = ringMorphologyEx(layer, size, Method::RING_CLOSE);
//     }

//     int regions = 0;
//     for (int i = 0; i < layer.size() - 1; i++) {
//         regions += std::abs(layer[i + 1] - layer[i]);
//     }
//     regions += std::abs(layer.back() - layer[0]);

//     return regions == 4;
// }

// std::vector<double> ringMorphologyEx(std::vector<double> img, int size, Method method) {
//     int stretch = size / 2;
//     std::vector<double> newImg = img;
//     if (method == Method::RING_DILATE) {
//         for (int i = 0; i < img.size(); i++) {
//             if (img[i] == 1) {
//                 for (int j = i - stretch; j <= i + stretch; j++) {
//                     int k = j % ((int)img.size());
//                     k += k < 0 ? img.size() : 0;
//                     if (img[k] == 0) {
//                         newImg[k] = 1;
//                     }
//                 }
//             }
//         }
//     } else if (method == Method::RING_ERODE) {
//         for (int i = 0; i < img.size(); i++) {
//             if (img[i] == 0) {
//                 for (int j = i - stretch; j <= i + stretch; j++) {
//                     int k = j % ((long int)img.size());
//                     k += k < 0 ? img.size() : 0;
//                     if (img[k] == 1) {
//                         newImg[k] = 0;
//                     }
//                 }
//             }
//         }
//     } else if (method == Method::RING_OPEN) {
//         newImg = ringMorphologyEx(img, size, Method::RING_ERODE);
//         newImg = ringMorphologyEx(newImg, size, Method::RING_DILATE);
//     } else if (method == Method::RING_CLOSE) {
//         newImg = ringMorphologyEx(img, size, Method::RING_DILATE);
//         newImg = ringMorphologyEx(newImg, size, Method::RING_ERODE);
//     }

//     return newImg;
// }