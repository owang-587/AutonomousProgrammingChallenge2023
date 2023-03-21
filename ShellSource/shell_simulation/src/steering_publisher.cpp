#include "shell_simulation/steering_publisher.h"

#include "std_msgs/String.h"
#include <sstream>

SteeringPublisher::SteeringPublisher() {
   string_pub_ = nh_.advertise<std_msgs::String>("steering", 1);
}

void SteeringPublisher::publish(std_msgs::Float64 msg) {
    string_pub_.publish(msg);
}
