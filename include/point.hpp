#include <vector>
#include <memory> 

/**
 * @brief State space point implementation
 */
class Point
{
public:

  // Constructor
  Point();
  Point(double _x,
        double _y,
        const double _parentX=0.0,
        const double _parentY=0.0,
        const std::vector<Point> _path={});

  double x;
  double y;
  double parentX;
  double parentY;
  std::vector<Point> path;
};
