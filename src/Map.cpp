#include <ras_group8_map/Map.hpp>
#include <nav_msgs/OccupancyGrid.h>

// STD
#include <string>

namespace ras_group8_map {

Map::Map(ros::NodeHandle& node_handle,
         int width,
         int height,
         double resolution,
         const std::string& frame_id,
         const std::string& map_topic,
         const std::string& service_topic)
    : node_handle_(node_handle)
{
  map_res_.map.info.map_load_time = ros::Time::now();
  map_res_.map.header.frame_id = frame_id;
  map_res_.map.header.seq = 0;
  
  map_res_.map.info.width = width;
  map_res_.map.info.height = height;
  map_res_.map.info.resolution = resolution;
  
  /* Allocate the map */
  map_res_.map.data.resize(width * height);
  
  /* TODO: Set map_res_.map.info.origin */
  // map_res_.map.info.origin.position
  // map_res_.map.info.origin.orientation
  
  /* Setup the map service */
  map_service_ =
    node_handle_.advertiseService(service_topic, &Map::mapServiceCallback,
                                  this);
  
  /* Publish the map */
  map_publisher_ =
    node_handle_.advertise<nav_msgs::OccupancyGrid>(map_topic, 1, true);
  map_publisher_.publish( map_res_.map );

  ROS_INFO("Successfully launched node.");
}

Map::~Map()
{
}

bool Map::mapServiceCallback(nav_msgs::GetMap::Request&  req,
                             nav_msgs::GetMap::Response& res)
{
  map_res_.map.header.seq  += 1;
  map_res_.map.header.stamp = ros::Time::now();
  
  /* Copy the map to the response. Apparently operator= is
     overloaded to do that. */
  res = map_res_;
  
  return true;
}

Map Map::load(ros::NodeHandle& n)
{
  std::string service_topic;
  std::string map_topic;
  std::string frame_id;
  int width;
  int height;
  double resolution;
  
  /* Load required parameters */
  if (!n.getParam("width", width))
    exit(ERROR_MISSING_PARAMETER);
    
  if (!n.getParam("height", height))
    exit(ERROR_MISSING_PARAMETER);
    
  if (!n.getParam("resolution", resolution))
    exit(ERROR_MISSING_PARAMETER);
  
  /* Load optional parameters */
  frame_id      = n.param("frame_id", std::string("map"));
  map_topic     = n.param("map_topic", std::string("map"));
  service_topic = n.param("service_topic", std::string("map"));
  
  Map map(n, width,
             height,
             resolution,
             frame_id,
             map_topic,
             service_topic);
  
  return map;
}


} /* namespace */