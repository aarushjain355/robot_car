#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <SoftwareSerial.h>
#include <RPLidar.h>
#include <ros_lib/path_planning/lidar_data.h>

ros::NodeHandle nh;
ros::Subscriber<std_msgs::String> lidarControl;
ros::Publisher liDARPub;
ros::Publisher distancesPub;
RPLidar lidar;
std_msgs::Float32MultiArray angles;
std_msgs::Float32MultiArray distances;
lidar_data::lidar_data lidar_msg;
Bool rotate;
SoftwareSerial imuSerial(2, 3);
unsigned long startTime;
unsigned long currentTime;
unsigned long elapsedTime;
int count = 0;


void lidarControlCallback(const std_msgs::String& msg) {

    if (msg.data == "Start") {

        rotate = true;
        startTime = millis();
        count = 0;
    }

}

void setup() {

    nh.initNode();
    Serial.begin(115200);
    imuSerial.begin(115200);
    lidar.setSerialBaudrate(115200);
    lidar.begin(Serial0);
    rotate = true;
    startTime = millis();
    lidarControl = nh.subscribe("lidar_control", 1, &lidarControlCallback);
    liDARPub = nh.advertise<lidar_data::lidar_data>("lidar_data", 1);
   

}

void loop() {

    currentTime = millis();
    elapsedTime = currentTime - startTime;

    if (rotate && elapsedTime < 10000) {

        if (IS_OK(lidar.waitPoint())) {

            float distance = lidar.getCurrentPoint().distance;
            float angle = lidar.getCurrentPoint().angle;
            bool startBit =lidar.getCurrentPoint().startBit;
        
            angles.push_back(angle);
            distances.push_back(distance);

        } 

    } else {
        if (count == 0) {
            lidar_msg.angles = angles;
            lidar_msg.distances = distances;
            angles.data.clear();
            distances.data.clear();
            liDARPub.publish(lidar_msg);
        }
        rotate = false;
        count = count + 1;
    }
    delay(100);
    nh.spinOnce();

}