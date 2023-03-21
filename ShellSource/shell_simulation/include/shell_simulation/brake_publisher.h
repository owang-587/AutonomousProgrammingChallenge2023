#include "ros/ros.h"

class BrakePublisher {
  public:
    BrakePublisher();

    void publish(std_msgs::Float64 msg);

  private:
    ros::NodeHandle nh_;

    ros::Publisher string_pub_;
};
