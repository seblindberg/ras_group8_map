#include <ros/ros.h>
#include <ras_group8_map/Map.hpp>

using namespace ras_group8_map;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ras_group8_map");
  ros::NodeHandle node_handle("~");

  Map main_object = Map::load(node_handle);

  ros::spin();
  return 0;
}