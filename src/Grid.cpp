#include <ras_group8_map/Grid.hpp>
#include <math.h>

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
Grid::drawPoint(nav_msgs::OccupancyGrid& grid,
                double x, double y, double p = 1.0)
{
  /* TODO: implement point drawing */
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
  ROS_ASSERT(p >= 0.0 && p <= 1.0);
    
  int cell_index = gridCellAt(row, col, grid);
  char q = p * 100.0;
  
  if (cell_index < 0) {
    return;
  }
  
  grid.data[cell_index] = q;
}

}