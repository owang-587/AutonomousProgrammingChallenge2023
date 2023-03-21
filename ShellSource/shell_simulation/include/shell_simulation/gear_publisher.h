#include "ros/ros.h"

class GearPublisher {
  public:
    GearPublisher();

    void publish(std_msgs::string msg);

  private:
    ros::NodeHandle nh_;

    ros::Publisher string_pub_;
};
