#include "shell_simulation/brake_publisher.h"

#include "std_msgs/String.h"
#include <sstream>

BrakePublisher::BrakePublisher() {
   string_pub_ = nh_.advertise<std_msgs::String>("brake", 1);
}

void BrakePublisher::publish(std_msgs::Float64 msg) {
    string_pub_.publish(msg);
}
