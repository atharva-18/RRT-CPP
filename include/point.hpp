#include <vector>
#include <memory> 

/**
 * @brief State space point implementation
 */
class Point
{
public:

  // Contructor
//   Point();

  // Destructor
//   ~Point();

  double x;
  double y;
  std::shared_ptr<Point> parent;
  std::vector<Point> path;
};
