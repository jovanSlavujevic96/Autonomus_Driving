cmake_minimum_required(VERSION 3.16.4)
project(bachelor)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport

  roscpp
  rospy

  message_generation
  sensor_msgs
  std_msgs
)

find_package(OpenCV 3 REQUIRED )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Coordinates.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs #sensros_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bachelorProject 
  CATKIN_DEPENDS cv_bridge image_transport roscpp rospy sensor_msgs std_msgs message_runtime
  DEPENDS #system_lib
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS} 
)

####### Data Protocol Lib ########################

add_library(_DataProtocolLib
  src/DataSender/DataSender.cpp
  src/DataReceiver/DataReceiver.cpp
)
############################################## CameraSimulator_Node

add_library(_CameraSimulatorLib
  src/CameraSimulator.cpp
)

add_executable(CameraSimulator_Node node/CameraSimulatorNode.cpp )

target_link_libraries(CameraSimulator_Node 
  _CameraSimulatorLib
  _DataProtocolLib 
  ${OpenCV_LIBS} 
  ${catkin_LIBRARIES} 
)
############################################## Cluster of Detector Nodes

add_library(_DetectorLib
  src/Detector.cpp  
  src/DetectorNode.cpp
)
####### Image Processor Lib #######

add_library(_ImageProcessorLib
  src/ImageProcessor/StopProcessor.cpp  
  src/ImageProcessor/LimitProcessor.cpp  
  src/ImageProcessor/LaneProcessor.cpp  
)
######################## LaneDetector_Node
add_executable(LaneDetector_Node node/detector_cluster/LaneDetectorNode.cpp ) 

target_link_libraries(LaneDetector_Node 
  _ImageProcessorLib
  _DetectorLib
  _DataProtocolLib
  ${OpenCV_LIBS} 
  ${catkin_LIBRARIES} 
)

add_dependencies(LaneDetector_Node 
  ${${PROJECT_NAME}_gencpp} 
  ${catkin_EXPORTED_TARGETS}
)
######################## StopDetector_Node
add_executable(StopDetector_Node node/detector_cluster/StopDetectorNode.cpp ) 

target_link_libraries(StopDetector_Node 
  _ImageProcessorLib
  _DetectorLib
  _DataProtocolLib
  ${OpenCV_LIBS} 
  ${catkin_LIBRARIES} 
)

add_dependencies(StopDetector_Node 
  ${${PROJECT_NAME}_gencpp} 
  ${catkin_EXPORTED_TARGETS}
)
######################## LimitDetector_Node
add_executable(LimitDetector_Node node/detector_cluster/LimitDetectorNode.cpp ) 

target_link_libraries(LimitDetector_Node 
  _ImageProcessorLib
  _DetectorLib
  _DataProtocolLib
  ${OpenCV_LIBS} 
  ${catkin_LIBRARIES} 
)

add_dependencies(LimitDetector_Node 
  ${${PROJECT_NAME}_gencpp} 
  ${catkin_EXPORTED_TARGETS}
)
############################################## Watchdog_Node 

add_library(_WatchdogLib
  src/Watchdog.cpp
)

add_executable(Watchdog_Node node/WatchdogNode.cpp )

target_link_libraries(Watchdog_Node 
  _WatchdogLib
  _DataProtocolLib 
  ${catkin_LIBRARIES} 
)

############################################## Visualizer_Node

add_library(_DisplayLib
  src/Display.cpp
)

add_library(_VisualizerLib
  src/Visualizer/LaneVisualizer.cpp
  src/Visualizer/ObjectVisualizer.cpp
)

add_executable(Visualizer_Node node/VisualizerNode.cpp ) 

target_link_libraries(Visualizer_Node 
  _VisualizerLib
  _DisplayLib
  _DataProtocolLib
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)

add_dependencies(Visualizer_Node 
  ${${PROJECT_NAME}_gencpp} 
  ${catkin_EXPORTED_TARGETS}
)

#################################################

add_executable(detTest test_node/detectionTest.cpp ) 

target_link_libraries(detTest 
  _DetectorLib
  _ImageProcessorLib
  ${OpenCV_LIBS} 
  ${catkin_LIBRARIES}
)

add_executable(detTest2 test_node/help.cpp ) 

target_link_libraries(detTest2 
  _DetectorLib
  _ImageProcessorLib
  ${OpenCV_LIBS} 
  ${catkin_LIBRARIES}
)

##############################################

#############
## Install ##
#############

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

find_package(catkin REQUIRED COMPONENTS roslaunch)
roslaunch_add_file_check(launch)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_bachelor.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
