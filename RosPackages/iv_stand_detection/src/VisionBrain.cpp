#include <VisionBrain.h>
#include <cv_bridge/cv_bridge.h>

VisionBrain::VisionBrain() {
    ros::NodeHandle nh;

    windowName = cv::String("view");
    // cv::namedWindow(windowName);

    image_transport::ImageTransport it{nh};
    imageSub = it.subscribe("/simon/camera_base_mount/simon/image_raw", 1, &VisionBrain::imageRecievedCallback, this);

    laserScanSub = nh.subscribe("/scan", 1, &VisionBrain::scanRecievedCallback, this);

    publisher = nh.advertise<std_msgs::Float64MultiArray>("/simon/vision/offsetCalc", 1);
}

VisionBrain::~VisionBrain() {
    // cv::destroyWindow(windowName);
}

void VisionBrain::imageRecievedCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        curFrame = cv_bridge::toCvShare(msg, "bgr8")->image;

        // cv::imshow(windowName, curFrame);
        // cv::waitKey(25);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void VisionBrain::scanRecievedCallback(const sensor_msgs::LaserScan::ConstPtr& scan) { curScan = *scan; }

void VisionBrain::executeTasks() {
    if (curFrame.empty()) {
        ROS_ERROR("No image recieved");
        return;
    }

    if (curScan.angle_min == 0.0) {
        ROS_ERROR("No scan recieved");
        return;
    }

    auto stand = StandAngleCalculator::calcStandAngle(curFrame, curScan);
    auto offset = OffsetCalculator::calcOffset(curFrame);

    auto message = constructMessage(stand, offset);
    publisher.publish(message);
}

std_msgs::Float64MultiArray VisionBrain::constructMessage(StandAngle stand, double offset) {
    std_msgs::Float64MultiArray result;

    auto h{1}, w{4};

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

    vec.push_back(static_cast<int>(stand.stand));
    vec.push_back(stand.angle);
    vec.push_back(stand.standDistance);
    vec.push_back(offset);

    result.data = vec;

    return result;
}