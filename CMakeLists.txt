cmake_minimum_required(VERSION 3.14.4)
project(manda_coverage)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(nav2_util REQUIRED)
find_package(project11_nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)


set(NODE_SOURCES 
    src/main.cpp
    src/PathPlan.cpp
    src/RecordSwath.cpp
    src/SurveyPath.cpp
    src/SurveyPath_Info.cpp
    src/lib_mbutil/MBUtils.cpp
    src/lib_mbutil/ColorParse.cpp
    src/lib_mbutil/ColorPack.cpp
    src/lib_mbutil/ReleaseInfo.cpp
    src/lib_geometry/XYObject.cpp
    src/lib_geometry/XYPoint.cpp
    src/lib_geometry/XYSegList.cpp
    src/lib_geometry/XYPolygon.cpp
    src/lib_geometry/XYVector.cpp
    src/lib_geometry/GeomUtils.cpp
    src/lib_geometry/AngleUtils.cpp
    src/lib_geometry/XYFormatUtilsSegl.cpp
    src/lib_geometry/XYPatternBlock.cpp
)

add_executable(${PROJECT_NAME}_node ${NODE_SOURCES})

target_include_directories(${PROJECT_NAME}_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/lib_geometry>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/lib_mbutil>
)

target_link_libraries(${PROJECT_NAME}_node
  ${project11_nav_msgs_TARGETS}
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
)

ament_target_dependencies(${PROJECT_NAME}_node
  Eigen3
  nav_2d_utils
  nav2_util
)

install (TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

#install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})


ament_package()

