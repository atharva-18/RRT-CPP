cmake_minimum_required(VERSION 3.7)
project(rrt)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-O2")

include_directories(.)
include_directories(include)
include_directories(External/json/include)
include_directories(External/matplotlib)

add_executable(rrt
               src/main.cpp
               src/rrt.cpp
               src/point.cpp
)

find_package(Python COMPONENTS Development)
target_include_directories(rrt PRIVATE ${Python_INCLUDE_DIRS})
target_link_libraries(rrt ${Python_LIBRARIES} )

