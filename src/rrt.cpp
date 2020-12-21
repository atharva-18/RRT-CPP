#include "rrt.hpp"
#include <iostream>

////////////////////////////////////////////////////////////////////////////////
RRT::RRT(Point _start,
         Point _end,
         std::vector<Obstacle> _obstacles,
         const double _maxStep /*2.0*/,
         const int _iterations /*=500 */,
         const int _sampleRangeMin /*=0*/,
         const int _sampleRangeMax /*=20*/,
         const double _precision /*=0.5*/)
{
  start = _start;
  end = _end;
  obstacles = _obstacles;
  maxStep = _maxStep;
  iterations = _iterations;
  sampleRangeMin = _sampleRangeMin;
  sampleRangeMax = _sampleRangeMax;
  precision = _precision;

  points.push_back(_start);
}

////////////////////////////////////////////////////////////////////////////////
bool RRT::run()
{
  int cnt = 0;
  for (int i = 0; i < iterations; i++, cnt++) {
    auto sampledPoint = sample();
    auto nearestPoint = getNearestPoint(sampledPoint);

    sampledPoint.parentX = nearestPoint.x;
    sampledPoint.parentY = nearestPoint.y;
    sampledPoint.path = nearestPoint.path;
    sampledPoint.path.push_back(sampledPoint);

    double distance = getDistance(sampledPoint, nearestPoint);
    bool colliding = collision(sampledPoint);

    if (distance > maxStep || colliding)
      continue;

    if (!colliding && distance <= maxStep) {
      points.push_back(sampledPoint);

      if(checkPathClosure(sampledPoint))
        return true;
    }
  }

  std::cout << "Iter: " << cnt << std::endl;

  return false;
}

////////////////////////////////////////////////////////////////////////////////
Point RRT::sample()
{ 
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> distr(sampleRangeMin, sampleRangeMax);
  Point sampledPoint = Point(static_cast<double>(distr(gen)), 
                             static_cast<double>(distr(gen)));

  return sampledPoint;
}

////////////////////////////////////////////////////////////////////////////////
Point RRT::getNearestPoint(Point _point)
{
  auto it = std::min_element(points.begin(), points.end(), [&](const Point &a, const Point &b) {
    return getDistance(_point, a) < getDistance(_point, b);
  });

  return *it;
}

////////////////////////////////////////////////////////////////////////////////
Obstacle RRT::getNearestObstacle(Point _point)
{
  auto it = std::min_element(obstacles.begin(), obstacles.end(), [&](const Obstacle &a, const Obstacle &b) {
    return getDistance(_point, a) < getDistance(_point, b);
  });

  return *it;
}

////////////////////////////////////////////////////////////////////////////////
template<typename A, typename B> double RRT::getDistance(A &_a, B &_b)
{
  double dx = _a.x - _b.x;
  double dy = _a.y - _b.y;
  double d = std::hypot(dx, dy);

  return d;
}

////////////////////////////////////////////////////////////////////////////////
std::vector<Point> RRT::getPath()
{
  return points[points.size() - 1].path;
}

////////////////////////////////////////////////////////////////////////////////
std::vector<Point> RRT::getPoints()
{
  return points;
}

////////////////////////////////////////////////////////////////////////////////
 bool RRT::collision(Point _point)
 {
    for (auto &obstacle:obstacles) {
      if (std::hypot(obstacle.x - _point.x, obstacle.y - _point.y) - obstacle.a < 1e-2)
        return true;

      const double dx = _point.x - _point.parentX;
      const double dy = _point.y - _point.parentY;
      const double d = std::hypot(dx, dy);

      const int nmAddPoints = d / precision;

      for (unsigned int i = 0; i < nmAddPoints; ++i) {
        double newX = _point.parentX;
        double newY = _point.parentY;
        newX += precision * i * dx / d;
        newY += precision * i * dy / d;
        
        const double dist = std::hypot(newX - obstacle.x, newY - obstacle.y);

        if (dist <= obstacle.a)
          return true;
      }
    }
    return false;
 }

////////////////////////////////////////////////////////////////////////////////
bool RRT::checkPathClosure(Point _point)
{
  double endDistance = getDistance(_point, end);

  if (endDistance <= maxStep) {
    Point endPoint = Point(end.x, end.y);
    endPoint.parentX = _point.x;
    endPoint.parentY = _point.y;
    endPoint.path = _point.path;
    endPoint.path.push_back(endPoint);

    if (!collision(endPoint)) {
      points.push_back(endPoint);
      return true;
    }
  }

  return false;
}
