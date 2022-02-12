#include <DateDetector.h>
#include <VisionBrain.h>
#include <cv_bridge/cv_bridge.h>

VisionBrain::VisionBrain() {
    ros::NodeHandle nh;

    windowName = cv::String("view");
    cv::namedWindow(windowName);

    image_transport::ImageTransport it{ nh };
    imageSub = it.subscribe("/usb_cam/image_raw", 1, &VisionBrain::imageRecievedCallback, this);

    dateGrabberPub = nh.advertise<std_msgs::Bool>("/simon/dateCollector/activate", 1);
    dateGrabberSub = nh.subscribe("/simon/dateCollector/isActive", 1, &VisionBrain::dateGrabberIsActiveCallback, this);
}

VisionBrain::~VisionBrain() {
    cv::destroyWindow(windowName);
}

void VisionBrain::imageRecievedCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        curFrame = cv_bridge::toCvShare(msg, "bgr8")->image;

        // cv::imshow(windowName, curFrame);
        // cv::waitKey(25);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void VisionBrain::dateGrabberIsActiveCallback(const std_msgs::Bool::ConstPtr& pumpFinished) {
    isDateGrabberActive = pumpFinished->data;
}

bool VisionBrain::executeTasks() {
    if (curFrame.empty()) {
        ROS_ERROR("No image recieved");
        return true;
    }
    auto prevTaskNumber = taskNumber;

    switch (taskNumber) {
    case 0:
        moveToDateArea();
        break;
    case 1:
        collectDates();
        break;
    case 2:
        moveBackUp();
        break;
    case 3:
        moveToNextDateLine();
        break;
    case 4:
        collectDates();
        break;
    case 5:
        moveBackUp();
        break;
    case 6:
        moveToNextDateLine();
        break;
    case 7:
        collectDates();
        break;
    case 8:
        moveBackUp();
        break;
    case 9:
        moveToStart();
        break;
    case 10:
        ROS_INFO("STOPPPED!!!");
        return false;
    }
    ROS_INFO("Task Number: %d!!!", taskNumber);

    if (taskNumber != prevTaskNumber) {
        counter = 0;
    }

    cv::imshow(windowName, curFrame);
    cv::waitKey(25);
    return true;
}

void VisionBrain::activateDateGrabber() {
    ROS_INFO("Activating Date Grabber");
    isDateGrabberActive = true;

    std_msgs::Bool result;
    result.data = true;

    dateGrabberPub.publish(result);
}

void VisionBrain::moveToDateArea() {
    ROS_INFO("Command: Move right by 1 metre");

    ros::Duration duration(5.);
    duration.sleep();

    taskNumber++;
}

void VisionBrain::printInstruction(cv::Point locationOffset) {
    std::string horizontal;
    std::string vertical;

    if (locationOffset == cv::Point(0, 0)) {
        ROS_INFO("Command: STOP!");
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

    ROS_INFO("Command: %s %d, %s, %d", horizontal.c_str(), abs(locationOffset.x), vertical.c_str(), abs(locationOffset.y));
}

void VisionBrain::collectDates() {

    if (counter >= 200 || numDates == 2) {
	numDates = 0;
        taskNumber++;
    }
    counter++;

    auto result = DateDetector::findDates(curFrame);
    if (result.size() == 0) {
        ROS_INFO("Command: Backwards");
        return;
    }

    auto offset = cv::Point((int)curFrame.cols / 2, (int)curFrame.rows / 2) - result[0];

    if (std::abs(offset.x) <= 100 && std::abs(offset.y) <= 100) {
        ROS_INFO("Command: Move back by 20 cm");

        ros::Duration duration(5.);
        duration.sleep();

        waitForCollectingDate();
    } else {
        printInstruction(offset);
    }
}

void VisionBrain::waitForCollectingDate() {
    activateDateGrabber();

    //while (isDateGrabberActive) {
   //     break;
   // }

    ros::Duration duration(70.);
    duration.sleep();


    taskNumber++;
}

void VisionBrain::moveBackUp() {
    ROS_INFO("Command: FORWARD");

    ros::Duration duration(10.);
    duration.sleep();

    ROS_INFO("Command: STOP");
    taskNumber++;
}

void VisionBrain::moveToNextDateLine() {
    ROS_INFO("Command: Move right by 50 cm");

    ros::Duration duration(5.);
    duration.sleep();

    taskNumber++;
}

void VisionBrain::moveToStart() {
    ROS_INFO("Command: Move left by 2 metres");

    ros::Duration duration(5.);
    duration.sleep();

    taskNumber++;
}
