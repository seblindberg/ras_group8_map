#include <ras_group8_map/Map.hpp>
#include <nav_msgs/OccupancyGrid.h>

// STD
#include <string>
#include <math.h>

namespace ras_group8_map {
  
Map::Map(ros::NodeHandle& node_handle,
         int width,
         int height,
         double resolution,
         const std::string& frame_id,
         const std::string& map_topic,
         const std::string& get_service_topic,
         const std::string& set_service_topic)
    : node_handle_(node_handle),
      scale_factor_(1.0 / resolution)
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
  
  /* Setup the map services */
  get_map_service_ =
    node_handle_.advertiseService(get_service_topic,
                                  &Map::getMapServiceCallback,
                                  this);
                                  
  set_map_service_ =
    node_handle_.advertiseService(set_service_topic,
                                  &Map::setMapServiceCallback,
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

/* Map Service Callback
 *
 * Called by the service server whenever a GetMap request is received. Returns
 * the preset mep response.
 */
bool Map::getMapServiceCallback(nav_msgs::GetMap::Request&  req,
                                nav_msgs::GetMap::Response& res)
{
  map_res_.map.header.seq  += 1;
  map_res_.map.header.stamp = ros::Time::now();
  
  /* Copy the map to the response. Apparently operator= is
     overloaded to do that. */
  res = map_res_;
  
  return true;
}

bool Map::setMapServiceCallback(nav_msgs::SetMap::Request&  req,
                                nav_msgs::SetMap::Response& res)
{
  /* Copy the map from the response. Apparently operator= is
     overloaded to do that. */
  map_res_.map = req.map;
  
  /* Consider reseting some of the header fields to known values */
  //map_res_.map.header.seq = 0;
  //map_res_.map.info.map_load_time = ros::Time::now();
  
  /* Publish the updated map */
  map_publisher_.publish(map_res_.map);
  
  res.success = true;

  return true;
}

Map Map::load(ros::NodeHandle& n)
{
  std::string service_topic;
  std::string update_topic;
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
  frame_id      = n.param("frame_id",      std::string("map"));
  map_topic     = n.param("map_topic",     std::string("map"));
  service_topic = n.param("service_topic", std::string("map"));
  update_topic  = n.param("update_topic",  std::string("update"));
  
  Map map(n, width,
             height,
             resolution,
             frame_id,
             map_topic,
             service_topic,
             update_topic);
  
  return map;
}

} /* namespace */