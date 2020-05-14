cmake_minimum_required(VERSION 2.8.3)
project(hans_bringup)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  geometry_msgs
   message_generation
  message_runtime
)
 add_message_files(
   FILES
   EncoderVel.msg
 )

 generate_messages(
   DEPENDENCIES
   std_msgs
 )


catkin_package(

)


include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
#add_library(AGVDriverlib
#  		src/TCPClient.cpp
#  		src/HansHardware.cpp
#)

add_library(AGVDriverlib
  		src/BoostTCPClient.cpp
  		src/AGVCmdFunction.cpp
)

add_library(ComDriverlib
  		src/SuperTerminal.cpp
)



add_executable(Odom_test_node src/Odom_test.cpp)
target_link_libraries(Odom_test_node AGVDriverlib ${catkin_LIBRARIES})

add_executable(com_ros_node src/BoostSerialDriver.cpp)
target_link_libraries(com_ros_node ComDriverlib ${catkin_LIBRARIES})

add_executable(com_test_node src/SuperTerminalTest.cpp)
target_link_libraries(com_test_node ComDriverlib ${catkin_LIBRARIES})

#add_executable(hans_bringup_node src/TcpClientTestNode.cpp)
#target_link_libraries(hans_bringup_node AGVDriverlib ${catkin_LIBRARIES})

#add_executable(hans_bringup_node2 src/TcpClientTestNode2.cpp)
#target_link_libraries(hans_bringup_node2 AGVDriverlib ${catkin_LIBRARIES})




## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(hans_bringup_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
## target_link_libraries(hans_bringup_node
##  ${catkin_LIBRARIES}
# )


