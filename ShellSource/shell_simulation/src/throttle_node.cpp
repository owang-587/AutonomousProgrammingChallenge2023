#include "shell_simulation/throttle_publisher.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "throttle");
    
    ThrottlePublisher node;

    ros::Rate rate(10);

    int count = 0;
    const int max_count = 100;


    //I'm not sure how we're supposed to control the throttle??
    while (ros::ok() && count < max_count) {
        /*
            I have edited the publish method to have input of std_msgs::Float64
            To input data must write message in msg.data where msg is of class std_msgs::Float64
            example:
                std_msgs::Float64 message
                message.data = 1.0
                node.publish(message)
        */
        node.publish();
        ros::spinOnce();
        rate.sleep();
        count++;
    }

    return 0;
}
