#pragma once

#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/SetMap.h>

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
      const std::string& get_service_topic,
      const std::string& set_service_topic);
  
  virtual
    ~Map();

  static Map
    load(ros::NodeHandle& node_handle);
    
private:
  int
    gridCellAt(int row, int col);
    
  int
    gridCellAt(double x, double y);
  
  void
    plot(int row, int col, double p);
  
  void
    drawLine(double x0, double y0, double x1, double y1);
    
  void
    drawLine(double x0, double y0, double x1, double y1, double thickness);
  
  bool
    getMapServiceCallback(nav_msgs::GetMap::Request&  req,
                          nav_msgs::GetMap::Response& res);
                          
  bool
    setMapServiceCallback(nav_msgs::SetMap::Request&  req,
                          nav_msgs::SetMap::Response& res);
                          
  /* ROS Objects
   */
  ros::NodeHandle& node_handle_;
 
  ros::Publisher map_publisher_;
 
  /* Services
   */
  ros::ServiceServer get_map_service_;
  ros::ServiceServer set_map_service_;
  
  /* Parameters
   */
  const double scale_factor_;
  
  /* Variables
   */
  //nav_msgs::OccupancyGrid map_msg_;
  nav_msgs::GetMap::Response map_res_;
};

} /* namespace */