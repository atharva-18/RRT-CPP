#include <vector>
#include <memory> 

/**
 * @brief State space point implementation
 */
class Obstacle
{
public:

  // Constructor
  Obstacle();
  Obstacle(double _x,
           double _y,
           double _a);

  double x;
  double y;
  double a;
};