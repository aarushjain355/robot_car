#include <ros/ros.h>
#include <std_msgs/String.h>


void subscriber_callback(const std_msgs::String::ConstPtr msg) {

    ROS_INFO("received message: %s", msg->data.c_str());
    
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "simple_subscriber_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("hand_gesture", 10, subscriber_callback);

}