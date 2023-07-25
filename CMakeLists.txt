cmake_minimum_required(VERSION 3.1.0)
project(test3d)

add_compile_options(-std=c++17)

find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

if (PCL_FOUND)
  include_directories(${PCL_INCLUDE_DIRS})
  add_definitions(${PCL_DEFINITIONS})
  link_directories(${PCL_LIBRARY_DIRS})
endif()

list(APPEND DEPENDENCIES ${PCL_LIBRARIES})

add_definitions(
  ${PCL_DEFINITIONS}
)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${realsense2_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  include
)
link_directories(
  ${PCL_LIBRARY_DIRS}
)


file(GLOB_RECURSE CPP_SOURCES src/*.cpp)

add_executable(${PROJECT_NAME}
  ${CPP_SOURCES}
)

target_link_libraries(${PROJECT_NAME}
  ${realsense2_LIBRARY}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)
