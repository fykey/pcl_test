cmake_minimum_required(VERSION 3.1.0)
project(test3d)

add_compile_options(-std=c++17)

find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (main2 src/main2.cpp)
target_link_libraries (main2 ${PCL_LIBRARIES}
  -lpthread
)
