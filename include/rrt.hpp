#include "point.hpp"
#include "obstacle.hpp"

#include <random>
#include <algorithm>
#include <cmath>
#include <vector>

/**
 * @brief RRT base class implementation from http://msl.cs.uiuc.edu/~lavalle/papers/Lav98c.pdf
 */
class RRT
{
public:
  // Constructor
  RRT();
  RRT(Point _start,
      Point _end,
      std::vector<Obstacle> _obstacles,
      const double _maxStep=3.0,
      const int _iterations=500,
      const int _sampleRangeMin=0,
      const int _sampleRangeMax=15,
      const double _precision=0.5);

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
   * @brief Checks if the given node collides with any obstacle
   * @param[in] _point Point to be checked for collision
   * @return True is colliding else false
   */
  bool collision(Point _point);

  /**
   * @brief Check if end point can be reached and add final point
   * @param[in] _point Penultimate point in the path
   * @return True is path is complete else false
   */
  bool checkPathClosure(Point _point);

  /**
   * @brief Get point that is nearest to a given point
   * @param[in] _point Point to be considered
   * @return Nearest Point
   */
  Point getNearestPoint(Point _point);

  /**
   * @brief Get obstacle that is nearest to a given point
   * @param[in] _point Point to be considered
   * @return Nearest Point
   */
  Obstacle getNearestObstacle(Point _point);

  /**
   * @brief Get distance between two points
   * @param[in] _a Point a
   * @param[in] _b Point b
   * @return double: Distance between two points
   */
  template<typename A, typename B> double getDistance(A &_a, B &_b);

  /**
   * @brief Get path from the last point
   * @return List of points in the path
   */
  std::vector<Point> getPath();

  /**
   * @brief Get all points in the state space
   * @return List of all points in the state space
   */
  std::vector<Point> getPoints();

private:
  Point start;
  Point end;
  std::vector<Point> points;
  std::vector<Obstacle> obstacles;
  int iterations;
  double sampleRangeMin;
  double sampleRangeMax;
  double maxStep;
  double precision;
};
