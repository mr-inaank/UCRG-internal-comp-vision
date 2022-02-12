#include <PoolDetector.h>
#include <VisionBrain.h>
#include <CheckerboardDetector.h>
#include <cv_bridge/cv_bridge.h>

VisionBrain::VisionBrain() {
    ros::NodeHandle nh;

    windowName = cv::String("view");
    cv::namedWindow(windowName);

    image_transport::ImageTransport it{ nh };
    imageSub = it.subscribe("/usb_cam/image_raw", 1, &VisionBrain::imageRecievedCallback, this);

    pumpPub = nh.advertise<std_msgs::Bool>("/uav_arduino/in", 1);
    pumpSub = nh.subscribe("/uav_arduino/out", 1, &VisionBrain::pumpIsActiveCallback, this);
}

VisionBrain::~VisionBrain() {
    cv::destroyWindow(windowName);
}

void VisionBrain::imageRecievedCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        curFrame = cv_bridge::toCvShare(msg, "bgr8")->image;

    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void VisionBrain::pumpIsActiveCallback(const std_msgs::Bool::ConstPtr& pumpFinished) {
    isPumpActive = pumpFinished->data;
}

bool VisionBrain::executeTasks() {
    if (curFrame.empty()) {
        ROS_ERROR("No image recieved");
        return true;
    }
    auto prevTaskNumber = taskNumber;

    printf("\nTask Number %d", taskNumber);
    switch (taskNumber) {
    case 0:
        takeoff();
        break;
    case 1:
        findPool();
        break;
    case 2:
        descendAboveZone();
        activatePump(true);
        isPumpActive = true;
        break;
    case 3:
        waitForWaterCollection();
        printf("Collecting Water...");
        break;
    case 4:
        takeoff();
        break;
    case 5:
        findCheckerBoard();
        break;
    case 6:
        descendAboveZone();
        activatePump(false);
        isPumpActive = true;
        break;
    case 7:
        waitForWaterCollection();
        printf("Discharging Water...");
        break;
    case 8:
        takeoff();
        break;
    case 9:
        returnToHome();
        break;
    case 10:
        printf("\nSTOPPPED!!!");
        return false;
    }

    if (taskNumber != prevTaskNumber) {
        counter = 0;
    }


    cv::imshow(windowName, curFrame);
    cv::waitKey(1);
    return true;
}

void VisionBrain::activatePump(bool activate) {
    std_msgs::Bool result;
    result.data = activate;

    pumpPub.publish(result);
}

void VisionBrain::takeoff() {
    if (counter < 25) {
        printf("\nCommand: UP");
        counter++;
        return;
    }

    taskNumber++;
    printf("\nCommand: STOP");
}

void VisionBrain::printInstruction(cv::Point locationOffset) {
    std::string horizontal;
    std::string vertical;

    if (locationOffset == cv::Point(0, 0)) {
        printf("\nCommand: STOP!");
    }

    if (locationOffset.x < 0) {
        horizontal = "LEFT ";
    } else if (locationOffset.x > 0) {
        horizontal = "RIGHT ";
    }

    if (locationOffset.y < 0) {
        vertical = "FORWARD ";
    } else if (locationOffset.y > 0) {
        vertical = "BACKWARD ";
    }

    printf("\nCommand: %s %d, %s, %d", horizontal.c_str(), abs(locationOffset.x), vertical.c_str(), abs(locationOffset.y));
}

void VisionBrain::findPool() {

    auto result = PoolDetector::getPoolOffset(curFrame);
    if (result == NULL_POINT) {
        printf("\nCommand: LEFT");
        return;
    }

    if (abs(result.x) <= 100 && abs(result.y) <= 100) {
        printf("\nCommand: STOP");
        taskNumber++;
    } else {
        printInstruction(result);
    }
}

void VisionBrain::descendAboveZone() {
    if (counter < 15) {
        printf("\nCommand: DOWN");
        counter++;
        return;
    }

    printf("\nCommand: STOP");
    // activatePump(true);
    // isPumpActive = true;
    taskNumber++;
}

void VisionBrain::waitForWaterCollection() {
    //taskNumber++;
    if (isPumpActive) {
        return;
    }

    taskNumber++;
}

void VisionBrain::findCheckerBoard() {
    auto result = CheckerboardDetector::getCheckerboardLocation(curFrame);
    if (result == NULL_POINT) {
        printf("\nCommand: LEFT");
        return;
    }


    result = cv::Point((int)curFrame.cols / 2, (int)curFrame.rows / 2) - result;

    if (abs(result.x) <= 100 && abs(result.y) <= 100) {
        printf("\nCommand: STOP");
        taskNumber++;
    } else {
        printInstruction(result);
    }

    return;
}

void VisionBrain::returnToHome() {
    if (counter < 30) {
        printf("\nCommand: LEFT");
        counter++;
        return;
    }

    if (counter < 50) {
        printf("\nCommand: DOWN");
        counter++;
        return;
    }
    printf("\nCommand: STOP");
    taskNumber++;
    return;
}
