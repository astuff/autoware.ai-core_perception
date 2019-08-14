cmake_minimum_required(VERSION 2.8.3)
project(deadreckoner)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  roslint
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
    std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

roslint_cpp()

add_executable(deadreckoner
  nodes/deadreckoner_node.cpp
  nodes/deadreckoner.cpp
)

target_link_libraries(deadreckoner
  ${catkin_LIBRARIES}
)

install(TARGETS
  deadreckoner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

if (CATKIN_ENABLE_TESTING)
  roslint_add_test()
endif ()
