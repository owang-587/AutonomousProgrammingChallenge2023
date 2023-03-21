#include "shell_simulation/gear_publisher.h"

#include "std_msgs/String.h"
#include <sstream>

GearPublisher::GearPublisher() {
   string_pub_ = nh_.advertise<std_msgs::String>("gear", 1);
}

void GearPublisher::publish(std_msgs::String msg) {
    string_pub_.publish(msg);
}
