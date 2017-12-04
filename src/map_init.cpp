#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/SetMap.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ras_group8_map/Grid.hpp>
#include <tf/LinearMath/Matrix3x3.h>

using namespace ras_group8_map;

static nav_msgs::SetMap set_srv;
static bool map_ok = false;

void
markerCallback(const visualization_msgs::MarkerArray& marker_array)
{
  /* TODO: Replace with Grid::drawMarkerArray */
  const int num_markers = marker_array.markers.size();
  
  /* Draw the lines in the grid */
  for (int i = 0; i < num_markers; i ++) {
    const visualization_msgs::Marker* marker = &marker_array.markers[i];
    
    /* Get the angle, distance and x,y coordinates for the
       marker and work out the (x0,y0) and (x1,y1)
       coordinates from that */
    const double d = marker->scale.x / 2.0;
    const double x = marker->pose.position.x;
    const double y = marker->pose.position.y;
    double a;
    double tmp;
    
    const tf::Quaternion q(marker->pose.orientation.x,
                           marker->pose.orientation.y,
                           marker->pose.orientation.z,
                           marker->pose.orientation.w);
    
    tf::Matrix3x3(q).getEulerYPR(a, tmp, tmp);
    
    const double d_cos_a = d * cos(a);
    const double d_sin_a = d * sin(a);
    
    const double x0 = x - d_cos_a;
    const double y0 = y - d_sin_a;
    const double x1 = x + d_cos_a;
    const double y1 = y + d_sin_a;
    
    Grid::drawLine(set_srv.request.map, x0, y0, x1, y1, 0.9);
    
    ROS_INFO("Drawing line (%f,%f) --- (%f,%f)", x0, y0, x1, y1);
  }
  
  
  map_ok = true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ras_group8_map_init");
  ros::NodeHandle node_handle("~");
  ros::Rate loop_rate(10.0);
  
  /* Load parameters */
  const std::string marker_topic =
    node_handle.param("marker_topic",      std::string("/maze_map"));
  const std::string get_service_topic =
    node_handle.param("get_service_topic", std::string("/map/map"));
  const std::string set_service_topic =
    node_handle.param("set_service_topic", std::string("/map/update"));
  
  /* Get the map in its current form */
  {
    ros::service::waitForService(get_service_topic, 10);
    
    nav_msgs::GetMap get_srv;
    ros::ServiceClient map_client =
      node_handle.serviceClient<nav_msgs::GetMap>(get_service_topic);
    
    if (!map_client.call(get_srv)) {
      ROS_ERROR("Failed to get the map");
      return 3;
    }
    
    /* Copy the map info */
    set_srv.request.map.header = get_srv.response.map.header;
    set_srv.request.map.info   = get_srv.response.map.info;
    set_srv.request.map.data.resize(get_srv.response.map.data.size());
    
  }
  
  ros::Subscriber marker_subscriber =
    node_handle.subscribe(marker_topic, 1, &markerCallback);
  
  /* Wait for a message with vizualization markers to come in */
  ROS_INFO("Waiting for marker array");
  while (!map_ok && ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  if (!ros::ok()) {
    return 0;
  }
  
  /* Update the map */
  {
    ros::ServiceClient map_client =
      node_handle.serviceClient<nav_msgs::SetMap>(set_service_topic);
      
    if (!map_client.call(set_srv)) {
      ROS_ERROR("Failed to set the map");
      return 3;
    }
  }
}