#include <VisionBrain.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "uav_water_collection");

    VisionBrain brain;
    ROS_INFO("READY1231234");

    while (ros::ok()) {
        brain.executeTasks();
        ros::spinOnce();
    }
}