#include <ras_group8_map/Map.hpp>
#include <nav_msgs/OccupancyGrid.h>

// STD
#include <string>
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

Map::Map(ros::NodeHandle& node_handle,
         int width,
         int height,
         double resolution,
         const std::string& frame_id,
         const std::string& map_topic,
         const std::string& get_service_topic,
         const std::string& set_service_topic)
    : node_handle_(node_handle),
      scale_factor_(1.0 / resolution)
{
  map_res_.map.info.map_load_time = ros::Time::now();
  map_res_.map.header.frame_id = frame_id;
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
  //map_res_.map.header.seq = 0;
  //map_res_.map.info.map_load_time = ros::Time::now();
  
  /* Publish the updated map */
  map_publisher_.publish(map_res_.map);
  
  res.success = true;

  return true;
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
int Map::gridCellAt(int row, int col)
{
  const int width = map_res_.map.info.width;
  
  /* Check that row and col are within the map
   * TODO: assert this check instead
   */
  if (col < 0 || col >= width ||
      row < 0 || row >= map_res_.map.info.height) {
    return -1;
  }
  
  return row * width + col;
}

/* Grid Cell At coordinates
 *
 * Returns the index of the grid cell at the specified x and y coordinates,
 * given in the map frame.
 *
 * It is assumed that the x axis is aligned with the columns of the grid and the
 * y axis with the rows.
 *
 * TODO: Account for offset?
 */
int Map::gridCellAt(double x, double y)
{
  int col = x * scale_factor_;
  int row = y * scale_factor_;
  
  return gridCellAt(row, col);
}

/* Plot
 * Writes a probability value [0,1] to the grid cell at the specified row and
 * column.
 */
void
  Map::plot(int row, int col, double p)
{
  ROS_ASSERT(p >= 0.0 && p <= 1.0);
  
  int cell_index = gridCellAt(row, col);
  
  ROS_ASSERT(cell_index >= 0);
  
  char q = p * 100.0;
  
  if (cell_index < 0) {
    return;
  }
  
  map_res_.map.data[cell_index] = q;
}

/* Draw Line
 *
 * Based on Xiaolin Wu's line algorithm. For details, see
 * https://en.wikipedia.org/wiki/Xiaolin_Wu's_line_algorithm.
 *
 * TODO: Only draw within the map.
 */
void
  Map::drawLine(double x0, double y0, double x1, double y1)
{
  bool steep = abs(y1 - y0) > abs(x1 - x0);
  
  x0 *= scale_factor_;
  y0 *= scale_factor_;
  x1 *= scale_factor_;
  y1 *= scale_factor_;
  
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
    const double xgap = rfpart(x0 + 0.5);
  
    xpxl1 = xend;
    ypxl1 = ipart(yend);
    
    if (steep) {
      plot(xpxl1, ypxl1    , rfpart(yend) * xgap);
      plot(xpxl1, ypxl1 + 1,  fpart(yend) * xgap);
    } else {
      plot(ypxl1,     xpxl1, rfpart(yend) * xgap);
      plot(ypxl1 + 1, xpxl1,  fpart(yend) * xgap);
    }
    
    intery = yend + gradient;
  }
    
  /* Second endpoint */
  {
    const double xend = round(x1);
    const double yend = y1 + gradient * (xend - x1);
    const double xgap = fpart(x1 + 0.5);
    
    xpxl2 = xend;
    ypxl2 = ipart(yend);
    
    if (steep) {
      plot(xpxl2, ypxl2,     rfpart(yend) * xgap);
      plot(xpxl2, ypxl2 + 1,  fpart(yend) * xgap);
    } else {
      plot(ypxl2    , xpxl2, rfpart(yend) * xgap);
      plot(ypxl2 + 1, xpxl2,  fpart(yend) * xgap);
    }
  }
  
  /* Main loop */
  if (steep) {
    for (xpxl1 ++; xpxl1 < xpxl2; xpxl1 ++) {
      plot(xpxl1, ipart(intery),    rfpart(intery));
      plot(xpxl1, ipart(intery) + 1, fpart(intery));
      intery += gradient;
    }
  } else {
    for (xpxl1 ++; xpxl1 < xpxl2; xpxl1 ++) {
      plot(ipart(intery)    , xpxl1, rfpart(intery));
      plot(ipart(intery) + 1, xpxl1,  fpart(intery));
      intery += gradient;
    }
  }
}

void Map::drawLine(double x0, double y0, double x1, double y1, double thickness)
{
  /* TODO: implement line drawing with thickness */
}

Map Map::load(ros::NodeHandle& n)
{
  std::string service_topic;
  std::string update_topic;
  std::string map_topic;
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
  
  Map map(n, width,
             height,
             resolution,
             frame_id,
             map_topic,
             service_topic,
             update_topic);
  
  return map;
}

int ipart(double x)
{
  return floor(x);
}

double round(double x)
{
  return floor(x + 0.5);
}

double fpart(double x)
{
  return x - floor(x);
}

double rfpart(double x)
{
  return 1 - fpart(x);
}


} /* namespace */