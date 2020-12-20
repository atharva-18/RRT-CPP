#include "obstacle.hpp"

////////////////////////////////////////////////////////////////////////////////
Obstacle::Obstacle(double _x,
                   double _y,
                   double _a)
{
  x = _x;
  y = _y;
  a = _a;
}

Obstacle::Obstacle()
: x(0.0), y(0.0), a(1.0)
{}
