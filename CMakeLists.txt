cmake_minimum_required(VERSION 3.5)

project(factory)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)

add_library(${CMAKE_PROJECT_NAME} SHARED src/factory.cpp)

ament_target_dependencies(${CMAKE_PROJECT_NAME}
  gazebo_ros
  rclcpp)

target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


message(STATUS "GAZEBO libraries : ${GAZEBO_LIBRARIES}")
target_link_libraries(${CMAKE_PROJECT_NAME} ${GAZEBO_LIBRARIES})

install(
  TARGETS ${CMAKE_PROJECT_NAME}
  EXPORT ${CMAKE_PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(
  DIRECTORY maps
  DESTINATION share/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
  
ament_package()