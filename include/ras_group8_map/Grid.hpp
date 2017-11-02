#pragma once

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

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
  
private:
  Grid() {};
  virtual ~Grid() {};
};

}