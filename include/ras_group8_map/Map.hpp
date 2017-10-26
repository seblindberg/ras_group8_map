#pragma once

#include <ros/ros.h>
#include <nav_msgs/GetMap.h>

namespace ras_group8_map {

#define ERROR_MISSING_PARAMETER 3

class Map
{
public:
  Map(ros::NodeHandle& node_handle,
      int width,
      int height,
      double resolution,
      const std::string& frame_id,
      const std::string& map_topic,
      const std::string& service_topic);
  
  virtual
    ~Map();

  static Map
    load(ros::NodeHandle& node_handle);
    
private:
  bool
    mapServiceCallback(nav_msgs::GetMap::Request&  req,
                       nav_msgs::GetMap::Response& res);
  /* ROS Objects
   */
  ros::NodeHandle& node_handle_;
 
  ros::Publisher map_publisher_;
 
  /* Services
   */
  ros::ServiceServer map_service_;
  
  /* Parameters
   */

  
  /* Variables
   */
  //nav_msgs::OccupancyGrid map_msg_;
  nav_msgs::GetMap::Response map_res_;
};

} /* namespace */