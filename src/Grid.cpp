#include <ras_group8_map/Grid.hpp>
#include <math.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/LinearMath/Matrix3x3.h>

namespace ras_group8_map {

/* Helper functions */
static inline int
  ipart(double x);

static inline double
  round(double x);

static inline double
  fpart(double x);

static inline double
  rfpart(double x);
  
static inline int
  gridCellAt(int row, int col, const nav_msgs::OccupancyGrid& grid);
  
static void
  plot(int row, int col, double p, nav_msgs::OccupancyGrid& grid);
  
static void
  drawLineHFast(int row, int col_0, int col_1, double p,
                nav_msgs::OccupancyGrid& grid);

static inline void
  drawLineMarker(nav_msgs::OccupancyGrid& grid,
                 const visualization_msgs::Marker& marker,
                 double p);
  
static inline void
  drawCircleMarker(nav_msgs::OccupancyGrid& grid,
                   const visualization_msgs::Marker& marker,
                   double p);

/* SPHERE = 2
 */

/* Draw Line
 *
 * Based on Xiaolin Wu's line algorithm. For details, see
 * https://en.wikipedia.org/wiki/Xiaolin_Wu's_line_algorithm.
 *
 * TODO: Check range of p.
 */
void
Grid::drawLine(nav_msgs::OccupancyGrid& grid,
               double x0, double y0, double x1, double y1,
               double p = 1.0)
{
  const double scale_factor = 1.0 / grid.info.resolution;
  const bool steep = fabs(y1 - y0) > fabs(x1 - x0);
  
  x0 *= scale_factor;
  y0 *= scale_factor;
  x1 *= scale_factor;
  y1 *= scale_factor;
  
  if (steep) {
    std::swap(x0, y0);
    std::swap(x1, y1);
  }
  
  if (x0 > x1) {
    std::swap(x0, x1);
    std::swap(y0, y1);
  }
  
  const double dx = x1 - x0;
  const double dy = y1 - y0;
  const double gradient = (dx == 0.0) ? 1.0 : dy / dx;
   
  int xpxl1;
  int ypxl1;
  int xpxl2;
  int ypxl2;
  double intery;
  
  /* First endpoint */
  {
    const double xend = round(x0);
    const double yend = y0 + gradient * (xend - x0);
    const double xgap = rfpart(x0 + 0.5) * p;
  
    xpxl1 = xend;
    ypxl1 = ipart(yend);
    
    if (steep) {
      plot(xpxl1, ypxl1    , rfpart(yend) * xgap, grid);
      plot(xpxl1, ypxl1 + 1,  fpart(yend) * xgap, grid);
    } else {
      plot(ypxl1,     xpxl1, rfpart(yend) * xgap, grid);
      plot(ypxl1 + 1, xpxl1,  fpart(yend) * xgap, grid);
    }
    
    intery = yend + gradient;
  }
    
  /* Second endpoint */
  {
    const double xend = round(x1);
    const double yend = y1 + gradient * (xend - x1);
    const double xgap = fpart(x1 + 0.5) * p;
    
    xpxl2 = xend;
    ypxl2 = ipart(yend);
    
    if (steep) {
      plot(xpxl2, ypxl2,     rfpart(yend) * xgap, grid);
      plot(xpxl2, ypxl2 + 1,  fpart(yend) * xgap, grid);
    } else {
      plot(ypxl2    , xpxl2, rfpart(yend) * xgap, grid);
      plot(ypxl2 + 1, xpxl2,  fpart(yend) * xgap, grid);
    }
  }
  
  /* Main loop */
  if (steep) {
    for (xpxl1 ++; xpxl1 < xpxl2; xpxl1 ++) {
      plot(xpxl1, ipart(intery),    rfpart(intery) * p, grid);
      plot(xpxl1, ipart(intery) + 1, fpart(intery) * p, grid);
      intery += gradient;
    }
  } else {
    for (xpxl1 ++; xpxl1 < xpxl2; xpxl1 ++) {
      plot(ipart(intery)    , xpxl1, rfpart(intery) * p, grid);
      plot(ipart(intery) + 1, xpxl1,  fpart(intery) * p, grid);
      intery += gradient;
    }
  }
}

void
Grid::drawLine(nav_msgs::OccupancyGrid& grid,
               double x0, double y0, double x1, double y1,
               double thickness, double p)
{
  /* TODO: implement line drawing with thickness */
}

void
Grid::drawCircle(nav_msgs::OccupancyGrid& grid,
                 double x0_f, double y0_f, double r_f, double p)
{
  const double scale_factor = 1.0 / grid.info.resolution;
  
  /* Scale everything */
  x0_f *= scale_factor;
  y0_f *= scale_factor;
  r_f *= scale_factor;
  
  /* Round everything */
  int x0 = round(x0_f);
  int y0 = round(y0_f);
  int r  = round(r_f);
  
  int x = r - 1;
  int y = 0;
  int dx = 1;
  int dy = 1;
  int err = dx - (r << 1);
  
  while (x >= y)
  {
    // plot(x0 - x, y0 + y, p, grid);
    // plot(x0 + x, y0 + y, p, grid);
    drawLineHFast(y0 + y, x0 - x, x0 + x, p, grid);
        
    // plot(x0 - y, y0 + x, p, grid);
    // plot(x0 + y, y0 + x, p, grid);
    drawLineHFast(y0 + x, x0 - y, x0 + y, p, grid);
    
    // plot(x0 - x, y0 - y, p, grid);
    // plot(x0 + x, y0 - y, p, grid);
    drawLineHFast(y0 - y, x0 - x, x0 + x, p, grid);
    
    // plot(x0 - y, y0 - x, p, grid);
    // plot(x0 + y, y0 - x, p, grid);
    drawLineHFast(y0 - x, x0 - y, x0 + y, p, grid);

    if (err <= 0) {
        y++;
        err += dy;
        dy += 2;
    }
    
    if (err > 0) {
        x--;
        dx += 2;
        err += dx - (r << 1);
    }
  }
}

void
Grid::drawPoint(nav_msgs::OccupancyGrid& grid,
                double x, double y, double p = 1.0)
{
  /* TODO: implement point drawing */
}

void
Grid::drawMarkerArray(nav_msgs::OccupancyGrid& grid,
                      const visualization_msgs::MarkerArray& marker_array,
                      double p_max)
{
  const int num_markers = marker_array.markers.size();
  
  /* Draw the lines in the grid */
  for (int i = 0; i < num_markers; i ++) {
    const visualization_msgs::Marker* marker = &marker_array.markers[i];
    const double rcomp = (marker->color).r;
    const double p = std::max(rcomp, p_max);
    ROS_INFO("p is %f, at pos %f %f", p, marker->pose.position.x, marker->pose.position.y);
    
    switch (marker->type) {
      case 2:
        drawCircleMarker(grid, *marker, p);
        break;
        
      default:
        drawLineMarker(grid, *marker, p);
    }
    
    // /* Get the angle, distance and x,y coordinates for the
    //    marker and work out the (x0,y0) and (x1,y1)
    //    coordinates from that */
    // const double d = marker->scale.x / 2.0;
    // const double x = marker->pose.position.x;
    // const double y = marker->pose.position.y;
    // double a;
    // double tmp;
    //
    // const tf::Quaternion q(marker->pose.orientation.x,
    //                        marker->pose.orientation.y,
    //                        marker->pose.orientation.z,
    //                        marker->pose.orientation.w);
    //
    // tf::Matrix3x3(q).getEulerYPR(a, tmp, tmp);
    //
    // const double d_cos_a = d * cos(a);
    // const double d_sin_a = d * sin(a);
    //
    // const double x0 = x - d_cos_a;
    // const double y0 = y - d_sin_a;
    // const double x1 = x + d_cos_a;
    // const double y1 = y + d_sin_a;
    //
    // drawLine(grid, x0, y0, x1, y1, p);
  }
}

void
drawLineMarker(nav_msgs::OccupancyGrid& grid,
               const visualization_msgs::Marker& marker,
               double p)
{
  /* Get the angle, distance and x,y coordinates for the
     marker and work out the (x0,y0) and (x1,y1)
     coordinates from that */
  const double d = marker.scale.x / 2.0;
  const double x = marker.pose.position.x;
  const double y = marker.pose.position.y;
  double a;
  double tmp;
  
  const tf::Quaternion q(marker.pose.orientation.x,
                         marker.pose.orientation.y,
                         marker.pose.orientation.z,
                         marker.pose.orientation.w);
  
  tf::Matrix3x3(q).getEulerYPR(a, tmp, tmp);
  
  const double d_cos_a = d * cos(a);
  const double d_sin_a = d * sin(a);
  
  const double x0 = x - d_cos_a;
  const double y0 = y - d_sin_a;
  const double x1 = x + d_cos_a;
  const double y1 = y + d_sin_a;
  
  Grid::drawLine(grid, x0, y0, x1, y1, p);
}

void
drawCircleMarker(nav_msgs::OccupancyGrid& grid,
                 const visualization_msgs::Marker& marker,
                 double p)
{
  const double r = marker.scale.x;
  const double x = marker.pose.position.x;
  const double y = marker.pose.position.y;
  
  Grid::drawCircle(grid, x, y, r, p);
}

nav_msgs::OccupancyGrid
Grid::downsample(const nav_msgs::OccupancyGrid& grid, int n)
{
  const int source_width  = grid.info.width;
  const int source_height = grid.info.height;
  const int target_width  = ceil((double) source_width / n);
  const int target_height = ceil((double) source_height / n);
  const int n_padding     = n - (target_width * n - source_width);
 
  /* Setup the down sampled grid */
  nav_msgs::OccupancyGrid target;
  
  target.info.width  = target_width;
  target.info.height = target_height;
  target.info.resolution = grid.info.resolution * n; // [m/cell]
  
  target.data.resize(target_width * target_height);
  
  /* Each row in the down sampled grid will be made up by a
     square of cells from the original. We can call them
     
     +------+------+------+------+------+------+
     | 2w+0 | 2w+1 | 2w+2 | 2w+3 | 2w+4 | 2w+5 |
     +------+------+------+------+------+------+
     |  w+0 |  w+1 |  w+2 |  w+3 |  w+4 |  w+5 | ...
     +------+------+------+------+------+------+
     |    0 |    1 |    2 |    3 |    4 |    5 |
     +------+------+------+------+------+------+
     
   */
  for (int r_target = 0; r_target < target_height; r_target ++) {
    const int target_row_offset = r_target * target_width;
    int source_row_offset = n * r_target * source_width;
    
    /* Initialize with the value of the cells in the lower
       left corner */
    int c_source = 0;
    for (int c_target = 0; c_target < target_width; c_target ++) {
      target.data[target_row_offset + c_target] =
        grid.data[source_row_offset + c_source];
      c_source += n; /* 0, n, 2n, ... */
    }
    
    /* Do the rest of the cells in the first row */
    for (int o = 1; o < n; o ++) {
      c_source = o;
      int c_target;
      for (c_target = 0; c_target < target_width - 1; c_target ++) {
        target.data[target_row_offset + c_target] =
          std::max(target.data[target_row_offset + c_target],
                   grid.data[source_row_offset + c_source]);
        c_source += n; /* o, n+o, 2n+o, ... o = [1,n) */
      }
    
      /* Do the last cell unless it actually does not exist
         in the source grid */
      if (o < n_padding) {
        target.data[target_row_offset + c_target] =
          std::max(target.data[target_row_offset + c_target],
                   grid.data[source_row_offset + c_source]);
      }
    }
    
    /* Do the other n-1 rows */
    for (int r_source = 1; r_source < n; r_source ++) {
      source_row_offset += source_width;
      for (int o = 0; o < n; o ++) {
        c_source = o;
        int c_target;
        for (c_target = 0; c_target < target_width; c_target ++) {
          target.data[target_row_offset + c_target] =
            std::max(target.data[target_row_offset + c_target],
                     grid.data[source_row_offset + c_source]);
          c_source += n; /* o, n+o, 2n+o, ... o = [0,n) */
        }
    
        /* Do the last cell unless it actually does not exist
           in the source grid */
        if (o < n_padding) {
          target.data[target_row_offset + c_target] =
            std::max(target.data[target_row_offset + c_target],
                     grid.data[source_row_offset + c_source]);
        }
      }
    }
  }
  
  return target;
}

/* Simulate Laser Scan
 *
 * Node: This is currently not working
 */
sensor_msgs::LaserScan
Grid::simulateLaserScan(const nav_msgs::OccupancyGrid& grid,
                  const double x, const double y, const double theta,
                  const int steps)
{
  const int width    = grid.info.width;
  const int height   = grid.info.height;
  const int grid_len = grid.data.size();
  const double cell_width = grid.info.resolution; /* [m/cell] */
  const double cell_center_offset = cell_width / 2;
  const double angle_increment = 2*M_PI / (steps + 1);
  const double range_index_scale = 1.0 / angle_increment;
  
  /* Our cell coordinates in the grid
     Assume we are within the grid for now */
  const double c0_real = x / cell_width;
  const double r0_real = y / cell_width;
  
  const int c0 = round(c0_real); /* Assume we are in the center of our cell */
  const int r0 = round(c0_real);
  
  sensor_msgs::LaserScan scan;
  
  scan.angle_increment = angle_increment;
  scan.angle_min = 0.0;
  scan.angle_max = angle_increment * steps;
  
  scan.range_min = 0.01; /* TODO: Should be > 0 */
  scan.range_max = 10.0; /* TODO: Ok to set arbitrary value? */
  
  scan.ranges.resize(steps);
  
  /* Perform ray-tracing */
  for (int i = 0; i < steps; i ++) {
    const double alpha = i * angle_increment;
    const double phi   = theta + alpha;
    
    const bool steep =
      (phi > (  M_PI / 4) && phi < (3*M_PI / 4))
                                || (phi > (5*M_PI / 4) && phi < (7*M_PI / 4));

    const double cos_phi = cos(phi);
    const double sin_phi = sin(phi);
    
    // ROS_INFO("cos_phi = %f, sin_phi = %f", cos_phi, sin_phi);
    
    if (steep) {
      const double d_grad  = 1.0 / sin_phi;
      
      if (sin_phi >= 0) {
        for (int r = 1; r < height - r0; r ++) {
          /* Distance to the cell in cell units */
          const double d = r * d_grad;
          const int c = (int) (d * cos_phi);
          
          if (c >= width) {
            scan.ranges[i] = d * cell_width;
            break;
          }
                  
          const int index = (r + r0) * width + (c + c0);
          
          if (grid.data[index] > 0) {
            scan.ranges[i] = d * cell_width;
            break; /* Leave the loop */
          }
        }
      } else {
        for (int r = 0; r < r0; r ++) {
          /* Distance to the cell in cell units */
          const double d = r * d_grad;
          const int c = (int) (d * cos_phi);
          
          if (c >= width) {
            scan.ranges[i] = d * cell_width;
            break;
          }
                  
          const int index = (-r + r0) * width + (c + c0);
          
          if (grid.data[index] > 0) {
            scan.ranges[i] = d * cell_width;
            break; /* Leave the loop */
          }
        }
      }
    } else {
      const double d_grad = 1.0 / cos_phi; /* How d changes for cells */
      
      if (cos_phi >= 0) {
        for (int c = 1; c < width - c0; c ++) {
          /* Distance to the cell */
          const double d = c * d_grad;
          const int r = (int) (d * sin_phi);
          
          if (r >= height) {
            scan.ranges[i] = d * cell_width;
            break;
          }
          
          const int index = (r + r0) * width + (c + c0);
          
          if (grid.data[index] > 0) {
            scan.ranges[i] = d * cell_width;
            break; /* Leave the loop */
          }
        }
      } else {
        for (int c = 1; c < c0; c ++) {
          /* Distance to the cell */
          const double d = -c * d_grad;
          const int r = (int) (d * sin_phi);
          
          if (r >= height) {
            scan.ranges[i] = d * cell_width;
            break;
          }
          
          const int index = (r + r0) * width + (-c + c0);
          
          if (grid.data[index] > 0) {
            scan.ranges[i] = d * cell_width;
            break; /* Leave the loop */
          }
        }
      }
    }
  }
  
  /* Iterate over every grid cell */
  // for (int i = 0; i < grid_len; i ++) {
  //   const char p = grid.data[i];
  //   /* Skip if the cell is not occupied */
  //   if (p < 1) {
  //     continue;
  //   }
  //
  //   const int    r = i / width;
  //   const int    c = i % width;
  //
  //   /* Get the world coordinates of the center of this cell */
  //   const double xi = cell_width * c + cell_center_offset;
  //   const double yi = cell_width * r + cell_center_offset;
  //
  //   const double dx = xi - x;
  //   const double dy = yi - y;
  //
  //   /* Calculate the angle relative to our position */
  //   const double phi = atan2(dy, dx);
  //
  //   /* Calculate the angle in the frame for the laser scanner */
  //   const double alpha = phi - theta;
  //   int          range_index = round(alpha * range_index_scale);
  //
  //   if (range_index >= steps) {
  //     ROS_ASSERT(range_index == steps);
  //     range_index = 0;
  //   }
  //
  //   const double d = sqrtf(dx*dx + dy*dy);
  //
  //   ROS_INFO("dx = %f", dx);
  //   ROS_INFO("dy = %f", dy);
  //   ROS_INFO("d = %f", d);
  //
  //   if (d < scan.ranges[range_index]) {
  //     scan.ranges[range_index] = d;
  //   }
  // }
  
  return scan;
}

int
ipart(double x)
{
  return floor(x);
}

double
round(double x)
{
  return floor(x + 0.5);
}

double
fpart(double x)
{
  return x - floor(x);
}

double
rfpart(double x)
{
  return 1 - fpart(x);
}

/* Grid Cell At indecies
 *
 * Returns the index of the grid cell at the specified row and column, or -1 if
 * the indecies are out of bounds.
 *
 * The map data is stored in row-major order, starting at (0,0). Example:
 *
 *   0:  M[0][0]
 *   1:  M[0][1]
 *   2:  M[1][0]
 *   …
 */
int
gridCellAt(int row, int col, const nav_msgs::OccupancyGrid& grid)
{
  const int width = grid.info.width;
  
  if (col < 0 || col >= width ||
      row < 0 || row >= grid.info.height) {
    return -1;
  }
  
  return row * width + col;
}

/* Plot
 * Writes a probability value [0,1] to the grid cell at the specified row and
 * column.
 */
void
plot(int row, int col, double p, nav_msgs::OccupancyGrid& grid)
{
  //ROS_INFO("p inside is %f, pos at %d %d ", p,row,col);
  //ROS_ASSERT(p >= 0.0 && p <= 1.0);
  
  if(p >= 0.0 && p <= 1.0)
  {
  int cell_index = gridCellAt(row, col, grid);
  char q = p * 100.0;
  
  if (cell_index < 0) {
    return;
  }
  
  grid.data[cell_index] = q;
}
}

void
drawLineHFast(int row, int col_0, int col_1, double p,
              nav_msgs::OccupancyGrid& grid)
{
  for (int col = col_0; col <= col_1; col++) {
    plot(row, col, p, grid);
  }
}

}
