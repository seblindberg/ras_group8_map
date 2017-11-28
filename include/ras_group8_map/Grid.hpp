#pragma once

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>

namespace ras_group8_map {

class Grid
{
public:
  static void
    drawLine(nav_msgs::OccupancyGrid& grid,
             double x0, double y0, double x1, double y1,
             double p);
             
  static void
    drawLine(nav_msgs::OccupancyGrid& grid,
             double x0, double y0, double x1, double y1,
             double thickness,
             double p);
             
  static void
    drawPoint(nav_msgs::OccupancyGrid& grid,
              double x, double y, double p);
              
  static sensor_msgs::LaserScan
    simulateLaserScan(const nav_msgs::OccupancyGrid& grid,
                      double x, double y, double theta, int resolution);
                      
  static nav_msgs::OccupancyGrid
    downsample(const nav_msgs::OccupancyGrid& grid, int n);
    
  
private:
  Grid() {};
  virtual ~Grid() {};
};

}