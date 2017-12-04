#pragma once

#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/SetMap.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>

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
      const std::string& set_service_topic,
      const std::string& marker_topic);
  
  virtual
    ~Map();

  static Map
    load(ros::NodeHandle& node_handle);
    
private:
  bool
    getMapServiceCallback(nav_msgs::GetMap::Request&  req,
                          nav_msgs::GetMap::Response& res);
                          
  bool
    setMapServiceCallback(nav_msgs::SetMap::Request&  req,
                          nav_msgs::SetMap::Response& res);
                          
  void
    markerArrayCallback(const visualization_msgs::MarkerArray& msg);
    
  void
    odomCallback(const nav_msgs::Odometry& msg);
                          
  // void
  //   pointCloudCallback(sensor_msgs::PointCloud& point_cloud);
  
  
  
  // void
  //   laserScanCallback(sensor_msgs::LaserScan& laser_scan);
          
  /* ROS Objects
   */
  ros::NodeHandle& node_handle_;
 
  ros::Publisher map_publisher_;
 
  /* Services
   */
  ros::ServiceServer get_map_service_;
  ros::ServiceServer set_map_service_;
  
  /* Subscribers
   */
  ros::Subscriber odom_subscriber_;
  ros::Subscriber point_subscriber_;
  ros::Subscriber laser_subscriber_;
  
  ros::Subscriber marker_subscriber_;
  
  /* Parameters
   */
  const double scale_factor_;
  const std::string frame_id_;
  
  /* Variables
   */
  //nav_msgs::OccupancyGrid map_msg_;
  nav_msgs::GetMap::Response map_res_;
};

} /* namespace */