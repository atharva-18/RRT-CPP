#include "rrt.hpp"

////////////////////////////////////////////////////////////////////////////////
RRT::RRT(Point _start, Point _end, std::vector<Point> _obstacles)
: iterations(500), threshold(0.5), sampleRangeMin(5), sampleRangeMax(20), maxStep(2.0)
{
  start = _start;
  end = _end;
  obstacles = _obstacles;
  points.push_back(_start);
}

////////////////////////////////////////////////////////////////////////////////
bool RRT::run()
{
  for(int i = 0; i < iterations; i++) {
    auto sampledPoint = sample();
    auto nearestPoint = getNearestPoint(sampledPoint);
    auto nearestObstacle = getNearestObstacle(sampledPoint);

    double distance = getDistance(sampledPoint, nearestPoint);
    double space = getDistance(sampledPoint, nearestObstacle);
    bool colliding = collision(sampledPoint);

    if (distance > maxStep || space < threshold || colliding)
      continue;

    if (!colliding && distance <= maxStep)
      points.push_back(sampledPoint);

      if(checkPathClosure(sampledPoint))
        return true;
  }

  return false;
}

////////////////////////////////////////////////////////////////////////////////
Point RRT::sample()
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> distr(sampleRangeMin, sampleRangeMax);
  
  Point sampledPoint = Point(distr(gen), distr(gen));

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
Point RRT::getNearestObstacle(Point _point)
{
  auto it = std::min_element(obstacles.begin(), obstacles.end(), [&](const Point &a, const Point &b) {
    return getDistance(_point, a) < getDistance(_point, b);
  });

  return *it;
}

////////////////////////////////////////////////////////////////////////////////
double RRT::getDistance(Point _a, Point _b)
{
  double dx = _a.x - _b.x;
  double dy = _a.y - _b.y;
  double d = std::hypot(dx, dy);

  return d;
}

////////////////////////////////////////////////////////////////////////////////
 bool RRT::collision(Point _point)
 {
    double min_distance = std::numeric_limits<double>::max();

    for(auto &obstacle:obstacles) {
      for(auto &point:_point.path) {
        double dx = obstacle.x - point.x;
        double dy = obstacle.y - point.y;

        double d = std::hypot(dx, dy);
        if(d < 1) {
          return true;  
        }
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
    endPoint.parent = std::shared_ptr<Point>(&_point);
    endPoint.path = _point.path;
    endPoint.path.push_back(endPoint);

    if(!collision(endPoint)) {
      points.push_back(endPoint);
      return true;
    }
  }

  return false;
}
