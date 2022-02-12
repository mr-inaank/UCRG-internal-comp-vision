#include <PoolDetector.h>
#include <VisionBrain.h>
#include <CheckerboardDetector.h>
#include <cv_bridge/cv_bridge.h>

VisionBrain::VisionBrain() {
    ros::NodeHandle nh;

    windowName = cv::String("view");
    cv::namedWindow(windowName);

    imageSub = nh.subscribe("iris/usb_cam/image_raw", 1, &VisionBrain::imageRecievedCallback, this);

    movePub = nh.advertise<std_msgs::Bool>("/move", 1);
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

void VisionBrain::move(int forwardBackward, int leftRight, int upDown, int yawLeftRight, int actuatorOpen) {
    std_msgs::Float64MultiArray result;

    auto h{ 1 }, w{ 5 };

    result.layout.dim.push_back(std_msgs::MultiArrayDimension());
    result.layout.dim[0].label = "height";
    result.layout.dim[0].size = h;
    result.layout.dim[0].stride = h * w;

    result.layout.dim.push_back(std_msgs::MultiArrayDimension());
    result.layout.dim[0].label = "width";
    result.layout.dim[0].size = w;
    result.layout.dim[0].stride = w;

    result.layout.data_offset = 0;

    std::vector<double> vec;

    std::string result = "Command: ";
    if (forwardBackward != 0) {
        result += forwardBackward == 1 ? "FORWARD" : "BACKWARD";
        result += " ";
    }
    if (leftRight != 0) {
        result += leftRight == 1 ? "LEFT" : "RIGHT";
        result += " ";
    }
    if (upDown != 0) {
        result += upDown == 1 ? "UP" : "DOWN";
        result += " ";
    }
    if (yawLeftRight != 0) {
        result += yawLeftRight == 1 ? "YAW_LEFT" : "YAW_RIGHT";
        result += " ";
    }
    if (actuatorOpen != 0) {
        result += actuatorOpen == 1 ? "ActuatorOpen" : "ActuatorClose";
        result += " ";
    }
    if (forwardBackward == 0 && leftRight == 0 && upDown == 0 && yawLeftRight == 0 && actuatorOpen == 0) {
        result += "STOP";
        result += " ";
    }

    vec.push_back(forwardBackward);         // Forward (1)/Back (-1)
    vec.push_back(leftRight);               // Left (1)/Right (-1)
    vec.push_back(upDown);                  // Up (1)/ Down (-1)
    vec.push_back(yawLeftRight);            // Yaw left (1)/ Yaw Right (-1)
    vec.push_back(actuatorOpen);            // Acuator Open (1)/Acuator Close (-1)

    result.data = vec;
    movePub.publish(result);
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
        break;
    case 3:
        waitForWaterCollection();
        break;
    case 4:
        takeoff();
        break;
    case 5:
        findCheckerBoard();
        break;
    case 6:
        descendAboveZone();
        break;
    case 7:
        waitForWaterCollection();
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


void VisionBrain::takeoff() {
    if (counter < 20) {
        move(0, 0, 1, 0, 0);
        counter++;
        return;
    }

    taskNumber++;
    move(0, 0, 0, 0, 0);
}

void VisionBrain::printInstruction(cv::Point locationOffset) {
    int horizontal;
    int vertical;

    if (locationOffset == cv::Point(0, 0)) {
        move(0, 0, 0, 0, 0);
    }

    if (locationOffset.x < 0) {
        horizontal = 1;
    } else if (locationOffset.x > 0) {
        horizontal = -1;
    }

    if (locationOffset.y < 0) {
        vertical = 1;
    } else if (locationOffset.y > 0) {
        vertical = -1;
    }

    printf("\nCommand: %s %d, %s, %d", horizontal.c_str(), abs(locationOffset.x), vertical.c_str(), abs(locationOffset.y));
    move(vertical, horizontal, 0, 0, 0);
}

void VisionBrain::findPool() {

    auto resultList = PoolDetector::getPoolOffset(curFrame);
    if (resultList.size() == 0) {
        move(1, 0, 0, 0, 0);
        return;
    }
    auto result = resultList[0];

    if (abs(result.x) <= 80 && abs(result.y) <= 80) {
        move(0, 0, 0, 0, 0);
        taskNumber++;
    } else {
        printInstruction(result);
    }
}

void VisionBrain::descendAboveZone() {
    if (counter < 15) {
        move(0, 0, -1, 0, 0);
        counter++;
        return;
    }

    move(0, 0, 0, 0, 0);
    activatePump(true);
    isPumpActive = true;
    taskNumber++;
}

void VisionBrain::waitForWaterCollection() {
    taskNumber++;
    if (isPumpActive) {
        return;
    }

    taskNumber++;
}

void VisionBrain::findCheckerBoard() {
    auto resultList = CheckerboardDetector::getCheckerboardLocation(curFrame);
    if (resultList.size() == 0) {
        move(0, 1, 0, 0, 0);
        return;
    }
    auto result = result[0];
    // Calculating offset vvvv
    result = cv::Point((int)curFrame.cols / 2, (int)curFrame.rows / 2) - result;

    if (abs(result.x) <= 30 && abs(result.y) <= 30) {
        move(0, 0, 0, 0, 0);
        taskNumber++;
    } else {
        printInstruction(result);
    }

    return;
}

void VisionBrain::returnToHome() {
    if (counter < 15) {
        move(0, 1, 0, 0, 0);
        counter++;
        return;
    }

    if (counter < 30) {
        move(0, 0, -1, 0, 0);
        counter++;
        return;
    }
    move(0, 0, 0, 0, 0);
    taskNumber++;
    return;
}