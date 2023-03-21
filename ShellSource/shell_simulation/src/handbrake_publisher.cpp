#include "shell_simulation/handbrake_publisher.h"

#include "std_msgs/String.h"
#include <sstream>

HandbrakePublisher::HandbrakePublisher() {
   string_pub_ = nh_.advertise<std_msgs::String>("handbrake", 1);
}

void HandbrakePublisher::publish(std_msgs::Bool msg) {
    string_pub_.publish(msg);
}
