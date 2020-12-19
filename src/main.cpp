#include "rrt.hpp"

#include <nlohmann/json.hpp>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;
using json = nlohmann::json;

int main() {
    plt::plot({1,3,2,4});
    plt::show();
}
