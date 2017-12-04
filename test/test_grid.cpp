#include <gtest/gtest.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <ras_group8_map/Grid.hpp>
#include <ras_group8_util/BMP.hpp>

using namespace ras_group8_map;

static void
  load_test_grid(nav_msgs::OccupancyGrid *grid);

TEST(Grid, test_simulate_laser_scanner)
{
  nav_msgs::OccupancyGrid grid;
  load_test_grid(&grid);
    
  const double x     = 0.3;
  const double y     = 0.3;
  const double theta = 0.0;
  
  sensor_msgs::LaserScan scan_data =
    Grid::simulateLaserScan(grid, x, y, theta, 360 - 1);
  
  const int num_ranges = scan_data.ranges.size();
  // ROS_INFO("num_ranges = %u", num_ranges);
  // for (int i = 0; i < num_ranges; i ++) {
  //   printf("%0.3f, ", scan_data.ranges[i]);
  // }
  //
  // printf("\n");
}

TEST(Grid, test_downsample)
{
  nav_msgs::OccupancyGrid grid;
  int res;
  load_test_grid(&grid);
  
  nav_msgs::OccupancyGrid grid_2x = Grid::downsample(grid, 2);
  nav_msgs::OccupancyGrid grid_3x = Grid::downsample(grid, 3);
  nav_msgs::OccupancyGrid grid_4x = Grid::downsample(grid, 13);
  
  FILE* f2 = fopen("/home/ras/catkin/maze_test_2x.bmp", "wb");
  FILE* f3 = fopen("/home/ras/catkin/maze_test_3x.bmp", "wb");
  FILE* f4 = fopen("/home/ras/catkin/maze_test_4x.bmp", "wb");
  
  if (res = ras_group8_util::BMP::write(grid_2x, f2)) {
    ROS_ERROR("Failed to write to file 2 (%i)", res);
    return;
  }
  
  if (res = ras_group8_util::BMP::write(grid_3x, f3)) {
    ROS_ERROR("Failed to write to file 3 (%i)", res);
    return;
  }
  
  if (res = ras_group8_util::BMP::write(grid_4x, f4)) {
    ROS_ERROR("Failed to write to file 4 (%i)", res);
    return;
  }
  
  if (f2 != NULL) fclose(f2);
  if (f3 != NULL) fclose(f3);
  if (f4 != NULL) fclose(f4);
}

TEST(Grid, test_draw_circle)
{
  nav_msgs::OccupancyGrid grid;
  int res;
  
  grid.data.resize(100);
  grid.info.with = 10;
  grid.info.height = 10;
  grid.info.resolution = 1;
  
  Grid::drawCircle(grid, 5, 5, 2, 0.7);
  
  FILE* f = fopen("/home/ras/catkin/grid_test_draw_cirlce.bmp", "wb");
  if (res = ras_group8_util::BMP::write(grid, f)) {
    ROS_ERROR("Failed to write to file (%i)", res);
    return;
  }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  srand((int)time(0));
  return RUN_ALL_TESTS();
}

void
load_test_grid(nav_msgs::OccupancyGrid *grid)
{
  FILE* f = fopen("/home/ras/catkin/maze_test.bmp", "rb");
  int res;
  
  if (f == NULL) {
    ROS_ERROR("Failed to open target file");
    return;
  }
  
  if (res = ras_group8_util::BMP::read(grid, f)) {
    ROS_ERROR("Failed to read from file (%i)", res);
  }
  
  if (f != NULL) fclose(f);
}