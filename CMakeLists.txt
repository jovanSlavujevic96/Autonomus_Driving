cmake_minimum_required(VERSION 2.8.3)
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
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

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
  sensor_msgs  std_msgs
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
  CATKIN_DEPENDS cv_bridge image_transport roscpp rospy sensor_msgs std_msgs
  DEPENDS system_lib
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

############################################## VideoPlayer_node
add_library(_VideoPlayerLib
  src/VideoPlayerNode/VideoPlayer.cpp
  src/DataProtocol/DataSender.cpp
  src/DataProtocol/DataReceiver.cpp
)

add_executable(VideoPlayerNode node/VideoPlayerNode.cpp )

target_link_libraries(VideoPlayerNode 
  _VideoPlayerLib 
  ${OpenCV_LIBS} 
  ${catkin_LIBRARIES} 
)

add_dependencies(VideoPlayerNode 
  ${${PROJECT_NAME}_gencpp} 
  ${catkin_EXPORTED_TARGETS}s
)
############################################## ObjectDetector_node

add_library(_ObjectDetectorLib
  src/ObjectDetectorNode/ObjectDetector.cpp  #main class (observer class)
  src/DataProtocol/DataSender.cpp
  src/DataProtocol/DataReceiver.cpp   #Observer Pattern (subject class)

  src/ObjectDetectorNode/StopSignProcessor.cpp  #Stop Sign detecting and drawing on frame
  src/ObjectDetectorNode/SpeedLimitProcessor.cpp  #Speed Limit Sign detecting and drawing on frame 	
)

add_executable(ObjectDetectorNode node/ObjectDetectorNode.cpp ) 

target_link_libraries(ObjectDetectorNode 
  _ObjectDetectorLib
  ${OpenCV_LIBS} 
  ${catkin_LIBRARIES} 
)

add_dependencies(ObjectDetectorNode 
  ${${PROJECT_NAME}_gencpp} 
  ${catkin_EXPORTED_TARGETS}
)

############################################## Timer_node

add_library(_TimerLib
  src/TimerNode/Timer.cpp
  src/DataProtocol/DataSender.cpp
  src/DataProtocol/DataReceiver.cpp
)

add_executable(TimerNode node/TimerNode.cpp )

target_link_libraries(TimerNode 
  _TimerLib 
  ${catkin_LIBRARIES} 
)

add_dependencies(TimerNode 
  ${${PROJECT_NAME}_gencpp} 
  ${catkin_EXPORTED_TARGETS}s
)

############################################## Watchdog 

add_library(_WatchdogLib
  src/WatchdogNode/Watchdog.cpp
  src/DataProtocol/DataReceiver.cpp
)

add_executable(WatchdogNode node/WatchdogNode.cpp )

target_link_libraries(WatchdogNode 
  _WatchdogLib 
  ${catkin_LIBRARIES} 
)

add_dependencies(WatchdogNode 
  ${${PROJECT_NAME}_gencpp} 
  ${catkin_EXPORTED_TARGETS}s 
)

############################################## Display node

add_library(_DisplayLib
  src/DisplayNode/Display.cpp
  src/DataProtocol/DataSender.cpp
  src/DataProtocol/DataReceiver.cpp
)

add_executable(DisplayNode node/DisplayNode.cpp )

target_link_libraries(DisplayNode 
  _DisplayLib
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES} 
)

add_dependencies(DisplayNode 
  ${${PROJECT_NAME}_gencpp} 
  ${catkin_EXPORTED_TARGETS}s 
)

##############################################

add_library(_Lib1
  src/ObjectDetectorNode/SpeedLimitProcessor.cpp
)

add_executable(LimitProc bzvz/test2/speedL.cpp )

target_link_libraries(LimitProc 
  _Lib1
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES} 
)

add_dependencies(LimitProc 
  ${${PROJECT_NAME}_gencpp} 
  ${catkin_EXPORTED_TARGETS}s 
)

##############################################

add_executable( StopDet bzvz/test3/STOPtest_det.cpp )

target_link_libraries(StopDet
  ${OpenCV_LIBS} 
  ${catkin_LIBRARIES} 
)

add_dependencies(StopDet 
  ${${PROJECT_NAME}_gencpp} 
  ${catkin_EXPORTED_TARGETS}s 
)

##############################################

add_executable( LaneDet bzvz/test1/main.cpp )

target_link_libraries(LaneDet
  ${OpenCV_LIBS} 
  ${catkin_LIBRARIES} 
)

add_dependencies(LaneDet 
  ${${PROJECT_NAME}_gencpp} 
  ${catkin_EXPORTED_TARGETS}s 
)

##############################################

add_executable( trainDigits bzvz/test4/trainDigits.cpp )

target_link_libraries(trainDigits
  ${OpenCV_LIBS} 
  ${catkin_LIBRARIES} 
)

add_dependencies(trainDigits 
  ${${PROJECT_NAME}_gencpp} 
  ${catkin_EXPORTED_TARGETS}s 
)

##############################################

add_executable( GenData bzvz/test5/GenData/GenData.cpp )

target_link_libraries(GenData
  ${OpenCV_LIBS} 
  ${catkin_LIBRARIES} 
)

add_dependencies(GenData 
  ${${PROJECT_NAME}_gencpp} 
  ${catkin_EXPORTED_TARGETS}s 
)

##############################################

add_executable( TrainAndTest bzvz/test5/TrainAndTest/TrainAndTest.cpp )

target_link_libraries(TrainAndTest
  ${OpenCV_LIBS} 
  ${catkin_LIBRARIES} 
)

add_dependencies(TrainAndTest 
  ${${PROJECT_NAME}_gencpp} 
  ${catkin_EXPORTED_TARGETS}s 
)

##############################################

add_executable( aaa bzvz/aaa.cpp )

target_link_libraries(aaa
  ${OpenCV_LIBS} 
  ${catkin_LIBRARIES} 
)

add_dependencies(aaa 
  ${${PROJECT_NAME}_gencpp} 
  ${catkin_EXPORTED_TARGETS}s 
)

####BZVZ


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
