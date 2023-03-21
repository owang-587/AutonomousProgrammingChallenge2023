#include "shell_simulation/handbrake_publisher.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "handbrake");
    
    HandbrakePublisher node;

    ros::Rate rate(10);

    int count = 0;
    const int max_count = 100;


    //I'm not sure how we're supposed to control the handbrake??
    while (ros::ok() && count < max_count) {
        /*
            I have edited the publish method to have input of std_msgs::Bool
            To input data must write message in msg.data where msg is of class std_msgs::Bool
            example:
                std_msgs::Bool message
                message.data = true
                node.publish(message)
        */
        node.publish();
        ros::spinOnce();
        rate.sleep();
        count++;
    }

    return 0;
}
