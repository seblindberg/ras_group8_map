#pragma once

#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>

namespace ras_group8_map {

class Map
{
public:
  Map(ros::NodeHandle& node_handle,
      int width,
      int height,
      double resolution,
      const std::string& service_topic);
  
  virtual
    ~Map();

  static Map
    load(ros::NodeHandle& node_handle);
    
private:
  bool
    mapServiceCallback(nav_msgs::GetMap::Request  &req,
                       nav_msgs::GetMap::Response &res);
  /* ROS Objects
   */
  ros::NodeHandle& node_handle_;
 
  /* Services
   */
  ros::ServiceServer map_service_;
  
  /* Parameters
   */
  const int width_;
  const int height_;
  const double resolution_; // [m/cell]
  
  /* Variables
   */
  nav_msgs::OccupancyGrid map_msg_;
  
  int8_t* map_;
};

} /* namespace */