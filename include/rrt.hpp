#include "point.hpp"

#include <random>
#include <algorithm>
#include <cmath>

/**
 * @brief RRT base class implementation from http://msl.cs.uiuc.edu/~lavalle/papers/Lav98c.pdf
 */
class RRT
{
public:
  // Constructor
  RRT();
  RRT(Point _start, Point _end, std::vector<Point> _obstacles);

  // Destructor
  ~RRT();

  /**
   * @brief Runs the RRT algorithm over given iterations
   * @return True is path is found else false
   */
  bool run();

  /**
   * @brief Samples a point in the state space.
   * @return Random Point in state space
   */
  Point sample();

  /**
   * @brief Get point that is nearest to a given point
   * @return Nearest Point
   */
  Point getNearestPoint(Point _point);

  /**
   * @brief Get obstacle that is nearest to a given point
   * @return Nearest Point
   */
  Point getNearestObstacle(Point _point);

  /**
   * @brief Returns a point considering source and destination
   * @return Point instance with path
   */
  Point getPoint(Point _source, Point _destination);

  /**
   * @brief Get distance between two points
   * @return double: Distance between two points
   */
  double getDistance(Point _a, Point _b);

  /**
   * @brief Checks if the given node collides with any obstacle
   * @return True is colliding else false
   */
  bool collision(Point _point);

  /**
   * @brief Check if end point can be reached and add final point
   * @return True is path is complete else false
   */
  bool checkPathClosure(Point _point);

  Point start;
  Point end;

private:
  std::vector<Point> points;
  std::vector<Point> obstacles;
  int iterations;
  double threshold;
  double sampleRangeMin;
  double sampleRangeMax;
  double maxStep;
};
