#include <ras_group8_map/Map.hpp>

// STD
#include <string>

namespace ras_group8_map {

Map::Map(ros::NodeHandle& node_handle,
         int width,
         int height,
         double resolution,
         const std::string& service_topic)
    : node_handle_(node_handle),
      width_(width),
      height_(height_),
      resolution_(resolution)
{
  /* Allocate the map */
  map_ = new int8_t[width * height];
    
  /* Setup the map service */
  map_service_ =
    node_handle_.advertiseService(service_topic, &Map::mapServiceCallback,
                                  this);

  ROS_INFO("Successfully launched node.");
}

Map::~Map()
{
  /* Deallocate the map */
  delete [] map_;
}

bool Map::mapServiceCallback(nav_msgs::GetMap::Request  &req,
                             nav_msgs::GetMap::Response &res)
{
  
}

Map Map::load(ros::NodeHandle& n)
{
  std::string service_topic;
  int width;
  int height;
  double resolution;
  
  if (!n.getParam("map_width", width))
    exit(3);
    
  if (!n.getParam("map_height", height))
    exit(3);
    
  if (!n.getParam("map_resolution", resolution))
    exit(3);
  
  service_topic = n.param("map_service_topic", std::string("map"));
    
  /* Setup a 10x10 grid where the side of each cell is 2.5 cm */
  Map map(n, width, height, resolution, service_topic);
  
  return map;
}


} /* namespace */