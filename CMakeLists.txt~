cmake_minimum_required(VERSION 2.8.3)
project(ras_group8_map)

find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  nav_msgs
  visualization_msgs
  sensor_msgs
  tf
  ras_group8_util
)

catkin_package(
  INCLUDE_DIRS
    include
    
  LIBRARIES
    ${PROJECT_NAME}_grid

  CATKIN_DEPENDS
    roscpp
    nav_msgs
    visualization_msgs
    sensor_msgs
    tf
    ras_group8_util

# DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#
# Grid
#

add_library(${PROJECT_NAME}_grid
  src/Grid.cpp
)

add_dependencies(${PROJECT_NAME}_grid
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_grid
  ${catkin_LIBRARIES}
)

if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}_grid_tests
    test/test_grid.cpp
  )

  target_link_libraries(${PROJECT_NAME}_grid_tests
    ${PROJECT_NAME}_grid
    ${catkin_LIBRARIES}
  )
endif()

#
# Map Node
#

add_executable(${PROJECT_NAME}
  src/map_node.cpp
  src/Map.cpp
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_init
  src/map_init.cpp
)

add_dependencies(${PROJECT_NAME}_init
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_init
  ${PROJECT_NAME}_grid
  ${catkin_LIBRARIES}
)

add_executable(maputil
  src/map_util.cpp
)

add_dependencies(maputil
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(maputil
  ras_group8_util_bmp
  ${catkin_LIBRARIES}
)



if (CATKIN_ENABLE_TESTING)
  #catkin_add_gtest(${PROJECT_NAME}_tests
  #  test/test_map.cpp
  #  src/Map.cpp
  #)
  
  #target_link_libraries(${PROJECT_NAME}_tests
  #  ${catkin_LIBRARIES}
  #)
endif()

# Make main executable
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  TARGETS ${PROJECT_NAME}_grid
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  TARGETS ${PROJECT_NAME}_init
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  TARGETS maputil
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install headers
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)