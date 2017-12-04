#include <ras_group8_map/Map.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <ras_group8_map/Grid.hpp>

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
         const std::string& set_service_topic,
         const std::string& marker_topic,
         const std::string& odom_topic)
    : node_handle_(node_handle),
      scale_factor_(1.0 / resolution),
      frame_id_(frame_id)
{
  map_res_.map.info.map_load_time = ros::Time::now();
  map_res_.map.header.frame_id = frame_id.c_str();
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
  
  marker_subscriber_ =
    node_handle_.subscribe(marker_topic, 1, &Map::markerArrayCallback, this);
  
  // point_subscriber_ =
  //   node_handle_.subscribe("points", 1, &Map::pointCloudCallback, this);
    
  /* Listen to updates from the odometry. */
  odom_subscriber_ =
    node_handle_.subscribe(odom_topic, 1,
                           &Map::odomCallback, this);
    
  // laser_subscriber_ =
  //   node_handle_.subscribe("points", 1, &Map::laserScanCallback, this);
  
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
  map_res_.map.header.seq = 0;
  map_res_.map.info.map_load_time = ros::Time::now();
  map_res_.map.header.frame_id = frame_id_;
  
  /* Publish the updated map */
  map_publisher_.publish(map_res_.map);
  
  res.success = true;

  return true;
}

void
Map::markerArrayCallback(const visualization_msgs::MarkerArray& marker_array)
{
  Grid::drawMarkerArray(map_res_.map, marker_array, 0.7);
}

void
Map::odomCallback(const nav_msgs::Odometry& msg)
{
  
}

/* Point Cloud Callback
 *
 * Update the map with a point cloud
 */
// void Map::pointCloudCallback(sensor_msgs::PointCloud& point_cloud)
// {
  // const int points_len   = point_cloud.points.size();
  // const int channels_len = point_cloud.channels.size();
  // int p_channel_i        = -1;
  
  /* TODO: Find the point_cloud.channel with the probability */
  // for (int i = 0; i < channels_len; i ++) {
  //   if (point_cloud.channels[i].name.compare("p") == 0) {
  //     p_channel_i = i;
  //     break;
  //   }
  // }
  
  // if (p_channel_i == -1) {
  //   ROS_ERROR("Could not find point probability channel in point cloud message");
  //   return;
  // }
  
  // for (int i = 0; i < points_len; i ++) {
  //   const double p = point_cloud.channels[p_channel_i][i];
  //   const double x = point_cloud.points[i].x;
  //   const double y = point_cloud.points[i].y;
  //
  //   ROS_INFO("Received point (%f, %f): %f", x, y, p);
  // }
// }

/* Laser Cloud Callback
 *
 * We don't want to say anything about the circle around our current location
 * with radius range_min.
 *
 * For each point we read
 */
// void Map::laserScanCallback(sensor_msgs::LaserScan& laser_scan)
// {
  // const double angle_increment = laser_scan.angle_increment;
  // const double range_min       = laser_scan.range_min;
  // const double range_max       = laser_scan.range_max;
  // const int ranges_len         = laser_scan.ranges.size();
  //
  // /* Get our current location from localization */
  //
  //
  // /* Iterate over the ranges */
  // for (int i = 0; i < ranges_len; i ++) {
  //   const double r = laser_scan.ranges[i];
  // }
// }

Map Map::load(ros::NodeHandle& n)
{
  std::string service_topic;
  std::string update_topic;
  std::string map_topic;
  std::string marker_topic;
  std::string odom_topic;
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
  marker_topic  = n.param("marker_topic",  std::string("markers"));
  odom_topic    = n.param("odom_topic",    std::string("odom"));
  
  Map map(n, width,
             height,
             resolution,
             frame_id,
             map_topic,
             service_topic,
             update_topic,
             marker_topic,
             odom_topic);
  
  return map;
}

} /* namespace */