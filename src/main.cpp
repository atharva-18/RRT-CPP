#include "rrt.hpp"

#include <nlohmann/json.hpp>
#include "matplotlibcpp.h"

#include <vector>
#include <fstream>
#include <cmath>

namespace plt = matplotlibcpp;
using json = nlohmann::json;

////////////////////////////////////////////////////////////////////////////////
int main()
{
  std::ifstream iConfig("../Params/config.json");
  json jsonConfig;
  iConfig >> jsonConfig;

  auto startPoint = jsonConfig["start"];
  auto endPoint = jsonConfig["end"];
  auto obs = jsonConfig["obstacles"];
  auto maxStep = jsonConfig["max_step"];
  auto iterations = jsonConfig["iterations"];
  auto sampleRangeMin = jsonConfig["sample_range_min"];
  auto sampleRangeMax = jsonConfig["sample_range_max"];
  auto precision = jsonConfig["precision"];

  Point start(startPoint[0], startPoint[1]);
  start.parentX = start.x;
  start.parentY = start.y;
  start.path.push_back(start);
  Point end(endPoint[0], endPoint[1]);
    
  std::vector<Obstacle> obstacles;

  for(auto _obs:obs) {
    obstacles.push_back(Obstacle(_obs[0], _obs[1], _obs[2]));
  }

  for(auto &obstacle: obstacles) {
    std::vector<double> ox;
    std::vector<double> oy;
    for (int deg = -1; deg < 360; deg += 5) {
      double rad = deg * M_PI/180.0;
      ox.push_back(obstacle.x + obstacle.a * std::cos(rad));
      oy.push_back(obstacle.y + obstacle.a * std::sin(rad));
    }

    plt::plot(ox, oy, "#0a192f");
  }

  RRT rrt(start, end, obstacles, maxStep, iterations, sampleRangeMin, sampleRangeMax, precision);

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
