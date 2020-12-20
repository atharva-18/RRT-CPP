#include "point.hpp"

////////////////////////////////////////////////////////////////////////////////
Point::Point(double _x,
             double _y,
             const double _parentX /*=0.0*/,
             const double _parentY /*=0.0*/,
             const std::vector<Point> _path /*={}*/)
{
  x = _x;
  y = _y;
  parentX = _parentX;
  parentY = _parentY;
  path = _path;
}

////////////////////////////////////////////////////////////////////////////////
Point::Point()
: x(0.0), y(0.0)
{}
