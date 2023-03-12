#include "ros/ros.h"
#include "std_msgs/String.h"

void colilisionCallback(const std_msgs::String:ConstPtr& msg){
    //msg is the string of what we input. So we can do with it as we need
    ROS_INFO("I heard [%s]", msg->data.c_str());
}

int main(int argc, char** argv){
    //Initialize the subscriber topic with the arguments argc,argv and named "collisionListener"
    ros::init(argc,argv,"collisionListener")
    //Create node instance of this topic
    ros::NodeHandle n;
    //Subscribe to topic and call colilisionCallback with the data received
    //Once we imuSub goes out of scope we unsubscribe
    ros::Subscriber imuSub = n.subscribe("/carla/ego_vehicle/imu", 100, colilisionCallback);
    //loops to keep calling back
    ros::spin();
}