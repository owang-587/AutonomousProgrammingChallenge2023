#include "ros/ros.h"

class HandbrakePublisher {
  public:
    HandbrakePublisher();

    void publish(std_msgs::Bool msg);

  private:
    ros::NodeHandle nh_;

    ros::Publisher string_pub_;
};
