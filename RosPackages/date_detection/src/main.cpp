#include <VisionBrain.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "date_collection");

    VisionBrain brain;
    ROS_INFO("READY1231234");

    ros::Rate rate(5);
    while (ros::ok()) {
        if (!brain.executeTasks()) {
            ros::spinOnce();
            break;
        }
        ros::spinOnce();
        rate.sleep();
    }
}