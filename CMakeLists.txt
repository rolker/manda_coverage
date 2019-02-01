cmake_minimum_required(VERSION 2.8.3)
project(manda_coverage)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  project11_transformations
)

find_path(EIGEN_INCLUDES Eigen/Core /usr/include/eigen3)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES asv_helm
  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
# include
  src/lib_geometry/
  src/lib_mbutil/
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDES}
)

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

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

