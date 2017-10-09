#pragma once

#include <ros/ros.h>
#include <phidgets/motor_encoder.h>

namespace ras_group8_template {

class Template
{
public:
  Template(ros::NodeHandle& node_handle);
  virtual ~Template();

private:
  bool readParameters();
  void topicCallback(const phidgets::motor_encoder& msg);

  /* ROS Objects
   */
  ros::NodeHandle& node_handle_;
  ros::Subscriber subscriber_;
  
  /* Parameters
   */
  std::string subscriber_topic_;
};

} /* namespace */