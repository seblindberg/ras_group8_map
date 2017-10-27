#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/SetMap.h>
#include <ras_group8_util/BMP.hpp>

#include <cstdio>
#include <string>

#define USAGE "Usage: \n" \
              "  maputil help\n"\
              "  maputil save [-f <filename>] service_topic\n" \
              "  maputil load [-f <filename>] service_topic"

int save(const std::string& service_topic, const std::string& filename)
{
  ros::NodeHandle node_handle("~");
  
  /* Request the map */
  ros::ServiceClient map_client =
    node_handle.serviceClient<nav_msgs::GetMap>(service_topic);
  
  nav_msgs::GetMap srv;
  if (map_client.call(srv)) {
    int res;
    FILE* f = fopen(filename.c_str(), "wb");
    
    if (f == NULL) {
      ROS_ERROR("Failed to open target file");
      return 1;
    }
    
    if (res = ras_group8_util::BMP::write(srv.response.map, f)) {
      ROS_ERROR("Failed to write to file (%i)", res);
      return 1;
    }
    
    fclose(f);
    
  } else {
    ROS_ERROR("Failed to fetch the map");
    return 1;
  }
  
  return 0;
}

int load(const std::string& service_topic, const std::string& filename)
{
  ros::NodeHandle node_handle("~");
  nav_msgs::SetMap srv;
  int res;
  
  FILE* f = fopen(filename.c_str(), "rb");
  
  if (f == NULL) {
    ROS_ERROR("Failed to open target file");
    return 1;
  }
  
  if (res = ras_group8_util::BMP::read(&srv.request.map, f)) {
    ROS_ERROR("Failed to read from file (%i)", res);
    return 1;
  }
  
  fclose(f);

  /* Set the map */
  ros::ServiceClient map_client =
    node_handle.serviceClient<nav_msgs::SetMap>(service_topic);
  
  if (map_client.call(srv)) {
    ROS_INFO("Successfully set the map");
  } else {
    ROS_ERROR("Failed to set the map");
    return 1;
  }

  return 0;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ras_group8_map_util");
  
  std::string service_topic;
  const std::string extname(".bmp");
  std::string filename("map");
  int argc_left;
  bool cmd_save;
    
  /* Check which command to use */
  if (argc < 2 || strcmp(argv[1], "help") == 0) {
    puts(USAGE);
    return 0;
  }
    
  if (argc >= 3 && strcmp(argv[1], "save") == 0) {
    cmd_save = true;
  } else if (argc >= 3 && strcmp(argv[1], "load") == 0) {
    cmd_save = false;
  } else {
    puts(USAGE);
    return 1;
  }
  
  argc_left = argc - 2;

  /* Parse arguments */
  for (int i = 2; i < argc; i++) {
    if (strcmp(argv[i], "-f") == 0) {
      if (++i < argc) {
        filename = argv[i];
        argc_left -= 2;
      } else {
        puts(USAGE);
        return 1;
      }
    }
  }
  
  if (argc_left < 1) {
    puts(USAGE);
    ROS_ERROR("No topic given");
    return 1;
  }
  
  service_topic = argv[argc - 1];
  
  /* Format the file name */
  if (filename.length() < extname.length() ||
      filename.find(extname, filename.length() - extname.length()) == std::string::npos)
  {
    filename = filename + extname;
  }
  
  if (cmd_save) {
    return save(service_topic, filename);
  } else {
    return load(service_topic, filename);
  }
}