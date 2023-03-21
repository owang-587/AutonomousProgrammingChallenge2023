#include "ros/ros.h"

class SteeringPublisher {
  public:
    SteeringPublisher();

    void publish(std_msgs::Float64 msg);

  private:
    ros::NodeHandle nh_;

    ros::Publisher string_pub_;
};
