cmake_minimum_required(VERSION 2.8.3)
project(erica_decision)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cmake_modules
  geometry_msgs
  message_generation
  erica_arm_module_msgs
  robotis_math
)



catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs 
  #DEPENDS system_lib

)
find_package(Eigen REQUIRED)

set(ENV{PKG_CONFIG_PATH} "$ENV{PKG_CONFIG_PATH}:${CATKIN_DEVEL_PREFIX}/lib/pkgconfig")
find_package(PkgConfig)
pkg_check_modules(yaml_cpp yaml-cpp REQUIRED)


include_directories(include ${catkin_INCLUDE_DIRS} )

add_executable(erica_decision_node src/erica_decision_node.cpp)
target_link_libraries(erica_decision_node ${catkin_LIBRARIES} ${yaml_cpp_LIBRARIES})
#add_dependencies(erica_decision_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

