cmake_minimum_required(VERSION 3.14.4)
project(manda_coverage)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(marine_nav_interfaces REQUIRED)
find_package(marine_nav_utilities REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)


add_library(${PROJECT_NAME} SHARED
  src/PathPlan.cpp
  src/RecordSwath.cpp
  src/SurveyPath.cpp
  src/lib_mbutil/MBUtils.cpp
  src/lib_mbutil/ColorParse.cpp
  src/lib_mbutil/ColorPack.cpp
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

target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_link_libraries(${PROJECT_NAME}
  marine_nav_utilities::marine_nav_utilities
  ${nav_msgs_TARGETS}
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
)

ament_target_dependencies(${PROJECT_NAME}
  nav2_util
  Eigen3
)


install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Eigen3
  sensor_msgs
  visualization_msgs
)


# Action server

add_executable(${PROJECT_NAME}_action_server 
  src/action_server.cpp
  src/main.cpp
)


target_link_libraries(${PROJECT_NAME}_action_server
  ${PROJECT_NAME}
  ${marine_nav_interfaces_TARGETS}
  marine_nav_utilities::marine_nav_utilities
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
)

install(TARGETS ${PROJECT_NAME}_action_server
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

