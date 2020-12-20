#include "rrt.hpp"

#include <nlohmann/json.hpp>
#include "matplotlibcpp.h"

#include <vector>

namespace plt = matplotlibcpp;
using json = nlohmann::json;

int main() {
  Point start(0.0, 0.0);
  start.parentX = start.x;
  start.parentY = start.y;
  start.path.push_back(start);
  Point end(6.0, 10.0);
    
  std::vector<Point> obstacles = {Point(5.0, 5.0),
                                  Point(3.0, 6.0),
                                  Point(3, 8),
                                  Point(3, 10),
                                  Point(7, 5),
                                  Point(9, 5),
                                  Point(8, 10)};

  std::vector<double> ox;
  std::vector<double> oy;

  for(auto &obstacle: obstacles) {
    ox.push_back(obstacle.x);
    oy.push_back(obstacle.y);
  }

  plt::scatter(ox, oy, 100.0);

  RRT rrt(start, end, obstacles);

  bool ok = rrt.run();

  auto path = rrt.getPath();
  auto points = rrt.getPoints();

  std::vector<double> pathX;
  std::vector<double> pathY;

  for (auto &point:path) {
    pathX.push_back(point.x);
    pathY.push_back(point.y);
  }

  for (auto &point:points) {
    std::vector<double> x = {point.x, point.parentX};
    std::vector<double> y = {point.y, point.parentY};

    // std::cout << "(" << x[0] << "," << y[0] << ")" << std::endl;
    // std::cout << "(" << x[1] << "," << y[1] << ")" << std::endl;

    plt::plot(x, y, "g");
    plt::pause(0.01);
  }

  if (ok) {
    std::cout << "Path size: " << rrt.getPath().size() << std::endl;
    std::cout << "Path found!" << std::endl;

    plt::plot(pathX, pathY, "r"); 
  } else {
    std::cout << "Path size: " << rrt.getPath().size() << std::endl;
    std::cout << "Algorithm time out...try again." << std::endl;
  }

  plt::show();
}
