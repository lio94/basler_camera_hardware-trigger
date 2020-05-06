cmake_minimum_required(VERSION 2.8.3)
project(basler_camera)

find_package(catkin REQUIRED COMPONENTS roscpp image_transport camera_info_manager cv_bridge)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES basler_camera
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

find_package(Pylon QUIET)
if (NOT ${Pylon_FOUND})
    include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/FindPylon.cmake")
endif()

###########
## Build ##
###########

## Additional Linker flags for pylon
SET( CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -Wl,-E")

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} $ENV{PYLON_ROOT}/include $ENV{GENICAM_ROOT_V2_3}/library/CPP/include)
include_directories(${Pylon_INCLUDE_DIRS})

link_directories(
  $ENV{PYLON_ROOT}/lib64 
  $ENV{GENICAM_ROOT_V2_3}/bin/Linux64_x64
  $ENV{GENICAM_ROOT_V2_3}/bin/Linux64_x64/GenApi/Generic
  $ENV{GENICAM_ROOT_V2_3}/library/CPP/lib/Linux64_x64
)

## Declare C++ executables
add_executable(basler_camera_node src/basler_camera_node.cpp src/basler_parameters.cpp)
add_dependencies(basler_camera_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_executable(basler_synced_cameras_node src/basler_synced_cameras_node.cpp src/basler_parameters.cpp)
add_dependencies(basler_synced_cameras_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


## Specify libraries to link a library or executable target against
target_link_libraries(basler_camera_node
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  ${Pylon_LIBRARIES}
  )
target_link_libraries(basler_synced_cameras_node
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  ${Pylon_LIBRARIES}
)
