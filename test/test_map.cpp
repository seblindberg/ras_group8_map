#include <gtest/gtest.h>
#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <ras_group8_map/Map.hpp>

using namespace ras_group8_map;

TEST(Map, test_map_service)
{
  ros::NodeHandle node_handle("~");
  Map map(node_handle,
          8,
          8,
          1.0,
          "map",
          "map");
          
  ros::ServiceClient service_client
    = node_handle.serviceClient<nav_msgs::GetMap>("map");
    
  nav_msgs::GetMap srv;
  
  if (service_client.call(srv)) {
    /* Inspect the response */
    EXPECT_EQ(srv.response.map.header.frame_id, std::string("map"));
    EXPECT_EQ(srv.response.map.header.seq, 1);
    
    EXPECT_EQ(srv.response.map.info.width, 8);
    EXPECT_EQ(srv.response.map.info.height, 8);
    EXPECT_FLOAT_EQ(srv.response.map.info.resolution, 1.0);
  } else {
    EXPECT_TRUE(false);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ras_group8_map_test");
  testing::InitGoogleTest(&argc, argv);
  srand((int)time(0));
  return RUN_ALL_TESTS();
}