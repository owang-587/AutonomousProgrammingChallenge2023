#include "shell_simulation/gear_publisher.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "gear");
    
    GearPublisher node;

    ros::Rate rate(10);

    int count = 0;
    const int max_count = 100;


    //I'm not sure how we're supposed to control the gear??
    while (ros::ok() && count < max_count) {
        /*
            I have edited the publish method to have input of std_msgs::string
            To input data must write message in msg.data where msg is of class std_msgs::string
            example:
                std_msgs::string message
                message.data = "forward"
                node.publish(message)
        */
        node.publish();
        ros::spinOnce();
        rate.sleep();
        count++;
    }

    return 0;
}
