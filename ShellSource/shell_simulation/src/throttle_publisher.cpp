#include "shell_simulation/throttle_publisher.h"

#include "std_msgs/String.h"
#include <sstream>

ThrottlePublisher::ThrottlePublisher() {
   string_pub_ = nh_.advertise<std_msgs::String>("throttle", 1);
}

void ThrottlePublisher::publish(std_msgs::Float64 msg) {
    string_pub_.publish(msg);
}
