#include "point.hpp"

#include <random>
#include <cmath>

/**
 * @brief RRT base class implementation from http://msl.cs.uiuc.edu/~lavalle/papers/Lav98c.pdf
 */
class RRT
{
public:
  // Constructor
  RRT(Point _start, Point _end);

  // Destructor
  ~RRT();

  /**
   * @brief Runs the RRT algorithm over given iterations
   */
  void run();

  /**
   * @brief Samples a point in the state space and adds it to the tree.
   */
  void sample();

  /**
   * @brief Checks if the given node collides with any obstacle
   * @return True is colliding else false
   */
  bool collision(Point _point);

  /**
   * @brief Get a point considering source and destination
   * @return Point instance with path
   */
  Point getPoint(Point _source, Point _destination);

  Point start;
  Point end;

private:
  int iterations;
  double threshold;
  double sampleRange;
  double maxStep;
};
